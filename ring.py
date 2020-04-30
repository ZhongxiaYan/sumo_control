#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# ring.py

from typing import List

from util import *
from sumo_util import *

class RingEnv:
    def __init__(self, c) -> None:
        self.c = c.setdefaults(redef_sumo=False)
        self.sumo_def = sumo_def = SumoDef(c)

        kwargs = self.def_sumo()
        kwargs['net'] = sumo_def.generate_net(**kwargs)
        sumo_def.sumo_cmd = sumo_def.generate_sumo(**kwargs)

        # Start traci
        self.tc = sumo_def.start_sumo()

        # Number of steps already executed
        self.steps: int = 0
        # self.tc.simulation.getCurrentTime() is useful for getting current simulation time

        self.init_vehicles()

        self.tc.simulationStep()
        self.steps += 1

    def def_sumo(self):
        c = self.c
        r = c.circumference / (2 * np.pi)
        # https://sumo.dlr.de/docs/Networks/PlainXML.html
        # https://sumo.dlr.de/docs/Definition_of_Vehicles,_Vehicle_Types,_and_Routes.html

        nodes = E('nodes',
            E('node', id='bottom', x=0, y=-r),
            E('node', id='top', x=0, y=r),
        )

        get_shape = lambda start_angle, end_angle: ' '.join('%.5f,%.5f' % (r * np.cos(i), r * np.sin(i)) for i in np.linspace(start_angle, end_angle, 80))
        edges = E('edges',
            E('edge', **{'id': 'right', 'from': 'bottom', 'to': 'top', 'length': c.circumference / 2, 'shape': get_shape(-np.pi / 2, np.pi / 2)}),
            E('edge', **{'id': 'left', 'from': 'top', 'to': 'bottom', 'length': c.circumference / 2, 'shape': get_shape(np.pi / 2, np.pi * 3 / 2)}),
        )

        connections = E('connections',
            E('connection', **{'from': 'left', 'to': 'right'}),
            E('connection', **{'from': 'right', 'to': 'left'}),
        )

        # IDM params
        v_params = {**IDM, **LC2013, **dict(accel=1, decel=1.5, minGap=2)}
        c.var(
            a=v_params['accel'],
            b=v_params['decel'],
            s0=v_params['minGap'],
            idm_tau=v_params['tau'],
            v0=v_params['maxSpeed'],
            delta=4
        )

        additional = E('additional',
            E('vType', id='idm', **v_params),
            E('route', id='route_right', edges='right left right'),
            E('route', id='route_left', edges='left right'),
            E('rerouter', E('interval', E('routeProbReroute', id='route_right'), begin=0, end=1e9), id='reroute', edges='right'),
        )
        args = {'no-internal-links': True}
        return self.sumo_def.save(nodes, edges, connections, additional, net_args=args, sumo_args=args)

    def init_vehicles(self) -> None:
        c = self.c

        curr: int = 0
        interval = c.circumference / c.n_veh
        self.veh_ids: List[str] = []
        assert c.n_veh % 2 == 0, f"{c.n_veh} (c.n_veh) must be even since half start on left and half start on right"
        # Vehicles have to follow a route, which consists of a sequence
        # of edges.
        # For example, route_right ("right left right") just says the
        # vehicle will start on edge right.
        for route, offset in ('route_right', 0), ('route_left', c.circumference / 2):
            for i in range(c.n_veh // 2):
                veh_id = f'v{curr}'
                self.tc.vehicle.add(veh_id, f'{route}',
                    typeID='idm',
                    departPos=str(max(0, curr * interval - offset + np.random.normal(0, 1))),
                    departSpeed='0')
                # disable sumo checks
                self.tc.vehicle.setSpeedMode(veh_id, SPEED_MODE.aggressive)
                curr += 1
                self.veh_ids.append(veh_id)
        assert curr == c.n_veh

    def step(self) -> None:
        c = self.c

        vehicle = self.tc.vehicle
        vehs = [Namespace(
            i=i,
            id=veh_id,
            edge=vehicle.getRoadID(veh_id),
            lane_pos=vehicle.getLanePosition(veh_id),
            speed=vehicle.getSpeed(veh_id),
            length=vehicle.getLength(veh_id)
        ) for i, veh_id in enumerate(self.veh_ids)]
        for veh in vehs:
            veh.pos = veh.lane_pos + (veh.edge == 'left') * c.circumference / 2

        if c.custom_update:
            for veh in vehs:
                next_veh = vehs[(veh.i + 1) % len(vehs)]

                s = next_veh.pos - veh.pos - veh.length
                if s < 0:
                    s += c.circumference
                v_diff = veh.speed - next_veh.speed
                s_star = c.s0 + veh.speed * c.idm_tau + veh.speed * v_diff / (2 * np.sqrt(c.a * c.b))
                veh.accel = c.a * (1 - (veh.speed / c.v0) ** c.delta - (s_star / s) ** 2)
                # verified that setSpeed sets speed immediately, slowDown linearly decelerates vehicle over duration
                vehicle.slowDown(veh.id, max(0, veh.speed + c.sim_step * veh.accel), duration=c.sim_step)

        speeds = [veh.speed for veh in vehs]
        print('Speed')
        print(' '.join(('%.2g' % speed) for speed in sorted(speeds)))
        print('mean %.5g min %.5g max %.5g' % (np.mean(speeds), np.min(speeds), np.max(speeds)))
        print()
        self.tc.simulationStep()
        self.steps += 1

        # if c.custom_update:
        #     for veh in vehs:
        #         custom_pos = veh.pos + c.sim_step * (veh.speed + c.sim_step * veh.accel * 0.5)
        #         if custom_pos >= c.circumference:
        #             custom_pos -= c.circumference
        #         sumo_pos = vehicle.getLanePosition(veh.id) + (vehicle.getRoadID(veh.id) == 'left') * c.circumference / 2
        #         print(veh.pos, custom_pos, sumo_pos)

if __name__ == '__main__':
    c = Namespace(
        res=Path('tmp'),
        horizon=3000,

        n_veh=22,
        circumference=250,
        sim_step=1,
        render=True,

        custom_update=True
    ).var(**from_args())
    env = RingEnv(c)
    for t in range(c.horizon):
        env.step()
