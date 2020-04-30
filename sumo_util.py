from lxml.etree import Element, SubElement, tostring, XMLParser
from xml.etree import ElementTree
import traci
import traci.constants as T
from traci.exceptions import FatalTraCIError, TraCIException
import bisect
import warnings

from typing import Any

from util import *

def val_to_str(x: Any) -> str:
    """
    If `x` is a boolean, then return "true" or "false".
    Otherwise, use the default conversion.

    :param x: Object to convert
    """
    return str(x).lower() if type(x) is bool else str(x)

def dict_kv_to_str(x):
    return dict(map(val_to_str, kv) for kv in x.items())

def str_to_val(x):
    for f in int, float:
        try:
            return f(x)
        except (ValueError, TypeError):
            pass
    return x

def values_str_to_val(x):
    for k, v in x.items():
        x[k] = str_to_val(v)
    return x

class E(list):
    xsi = 'http://www.w3.org/2001/XMLSchema-instance'
    root_args = dict(nsmap=dict(xsi=xsi))
    def __init__(self, _name, *args, **kwargs):
        assert all(type(a) is E for a in args)
        super().__init__(args)
        self._dict = kwargs
        self._name = _name

    def keys(self):
        return self._dict.keys()

    def values(self):
        return self._dict.values()

    def items(self):
        return self._dict.items()

    def __getitem__(self, k):
        if type(k) in [int, slice]:
            return super().__getitem__(k)
        return self._dict.__getitem__(k)

    def __setitem__(self, k, v):
        if type(k) in [int, slice]:
            return super().__setitem__(k, v)
        return self._dict.__setitem__(k, v)

    def __getattr__(self, k):
        if k.startswith('_'):
            return self.__dict__[k]
        else:
            return self[k]

    def __setattr__(self, k, v):
        if k.startswith('_'):
            self.__dict__[k] = v
        else:
            self[k] = v

    def __repr__(self):
        return self.to_string().decode()

    def to_element(self, root=True):
        e = Element(self._name, attrib={k: val_to_str(v) for k, v in self.items()}, **(E.root_args if root else {}))
        e.extend([x.to_element(root=False) for x in self])
        return e

    def to_string(self):
        return tostring(self.to_element(), pretty_print=True, encoding='UTF-8', xml_declaration=True)

    def to_path(self, p):
        p.save_bytes(self.to_string())

    def children(self, tag):
        return [x for x in self if x._name == tag]

    @classmethod
    def from_element(cls, e):
        return E(e.tag, *(cls.from_element(x) for x in e), **e.attrib)

    @classmethod
    def from_path(cls, p):
        return cls.from_element(ElementTree.parse(p, parser=XMLParser(recover=True)).getroot())

SPEED_MODE = Namespace(
    aggressive=0,
    obey_safe_speed=1,
    no_collide=7,
    right_of_way=25,
    all_checks=31
)

LC_MODE = Namespace(off=0, no_lat_collide=512, strategic=1621)

# Traffic light defaults
PROGRAM_ID = 1
MAX_GAP = 3.0
DETECTOR_GAP = 0.6
SHOW_DETECTORS = True

IDM = dict(
    accel=2.6,
    decel=4.5,
    sigma=0.5,
    tau=1.0,  # past 1 at sim_step=0.1 you no longer see waves
    minGap=2.5,
    maxSpeed=30,
    speedFactor=1.0,
    speedDev=0.1,
    impatience=0.5,
    carFollowModel='IDM',
)

LC2013 = dict(
    laneChangeModel='LC2013',
    lcStrategic=1.0,
    lcCooperative=1.0,
    lcSpeedGain=1.0,
    lcKeepRight=1.0,
)

SL2015 = dict(
    laneChangeModel='SL2015',
    lcStrategic=1.0,
    lcCooperative=1.0,
    lcSpeedGain=1.0,
    lcKeepRight=1.0,
    lcLookAheadLeft=2.0,
    lcSpeedGainRight=1.0,
    lcSublane=1.0,
    lcPushy=0,
    lcPushyGap=0.6,
    lcAssertive=1,
    lcImpatience=0,
    lcTimeToImpatience=float('inf'),
    lcAccelLat=1.0,
)

def FLOW(id, type, route, departSpeed, departLane='random', vehsPerHour=None, probability=None, period=None, number=None):
    flow = dict(
        id=id,
        type=type,
        route=route,
        departSpeed=departSpeed,
        departLane=departLane,
        begin=1
    )
    flow.update(dict(number=number) if number else dict(end=86400))
    if vehsPerHour:
        flow['vehsPerHour'] = vehsPerHour
    elif probability:
        flow['probability'] = probability
    elif period:
        flow['period'] = period
    return flow

# Vehicle colors
WHITE = (255, 255, 255)
CYAN = (0, 255, 255)
RED = (255, 0, 0)

COLLISION = Namespace(teleport='teleport', warn='warn', none='none', remove='remove')

class SumoDef:
    no_ns_attr = '{%s}noNamespaceSchemaLocation' % E.xsi
    xsd = 'http://sumo.dlr.de/xsd/%s_file.xsd' # For all other defs except viewsettings
    config_xsd = 'http://sumo.dlr.de/xsd/sumoConfiguration.xsd'
    # see https://sumo.dlr.de/docs/NETCONVERT.html
    netconvert_args = dict(nodes='n', edges='e', connections='x', types='t')
    config_args = dict(
        net='net-file', routes='route-files',
        additional='additional-files', gui='gui-settings-file'
    )
    file_args = set(netconvert_args.keys()) | set(config_args.keys())

    def __init__(self, c):
        self.c = c
        self.dir = (c.res / 'sumo').mk()
        self.sumo_cmd = None

    def save(self, *args, **kwargs):
        for e in args:
            e[SumoDef.no_ns_attr] = SumoDef.xsd % e._name
            kwargs[e._name] = path = self.dir / e._name[:3] + '.xml'
            e.to_path(path)
        return Namespace(**kwargs)

    def generate_net(self, **kwargs):
        net_args = Namespace(**kwargs.get('net_args', {})).setdefaults(**{
            'no-turnarounds': True
        })

        # https://sumo.dlr.de/docs/NETCONVERT.html
        net_path = self.dir / 'net.xml'
        args = ['netconvert', '-o', net_path]
        for name, arg in SumoDef.netconvert_args.items():
            path = kwargs.pop(name, None)
            if path:
                args.append('-%s %s' % (arg, path))
        args.extend('--%s %s' % (k, val_to_str(v)) for k, v in net_args.items())

        cmd = ' '.join(args)
        print(cmd)
        out, err = shell(cmd, stdout=None)
        if err:
            print(err)

        return net_path

    def generate_sumo(self, **kwargs):
        c = self.c

        gui_path = self.dir / 'gui.xml'
        E('viewsettings',
            E('scheme', name='real world'),
            E('background',
                backgroundColor='100,100,100',
                showGrid='0',
                gridXSize='100.00',
                gridYSize='100.00'
            )).to_path(gui_path)
        kwargs['gui'] = gui_path

        # https://sumo.dlr.de/docs/SUMO.html
        sumo_args = Namespace(
            **{arg: kwargs[k] for k, arg in SumoDef.config_args.items() if k in kwargs},
            **kwargs.get('sumo_args', {})).setdefaults(**{
            'begin': 0,
            'num-clients': 1,
            'step-length': c.sim_step,
            'no-step-log': True,
            'time-to-teleport': -1,
            'no-warnings': not c.get('print_warnings'),
            'collision.action': COLLISION.remove,
            'collision.check-junctions': True,
            'max-depart-delay': 0.5
        })
        cmd = ['sumo-gui' if c.render else 'sumo']
        for k, v in sumo_args.items():
            cmd.extend(['--%s' % k, val_to_str(v)] if v is not None else [])
        print(' '.join(cmd))
        return cmd

    def load_sumo(self):
        traci.load(self.sumo_cmd[1:])

    def start_sumo(self):
        traci.start(self.sumo_cmd)
        return traci.getConnection()
