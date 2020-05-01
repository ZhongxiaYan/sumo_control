#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# ring.py

from abc import ABC, abstractmethod
import collections
from dataclasses import dataclass
from typing import Dict, List, Set

from util import *
from sumo_util import *

@dataclass
class IDMParams:
    a: float
    b: float
    s0: float
    tau: float
    v0: float
    delta: float

class ControlLogic(ABC):
    """
    A block of logic that computes accelerations.
    """
    def __init__(self) -> None:
        self.steps: int = 0

    def step(self,
        distance_to_next_vehicle: float,
        this_speed: float,
        next_speed: float
    ) -> float:
        """
        Wraps step_logic with some additional logic, like incrementing
        the step counter.
        """
        return_value = self.step_logic(
            distance_to_next_vehicle=distance_to_next_vehicle,
            this_speed=this_speed,
            next_speed=next_speed
        )
        self.steps += 1
        return return_value

    @abstractmethod
    def step_logic(self,
        distance_to_next_vehicle: float,
        this_speed: float,
        next_speed: float
    ) -> float:
        """
        Run a step of this control logic and return acceleration of the
        car as a float.
        """
        pass

class CustomIDM(ControlLogic):
    def __init__(self, params: IDMParams) -> None:
        super().__init__()
        self.params: IDMParams = params

    def step_logic(self,
        distance_to_next_vehicle: float,
        this_speed: float,
        next_speed: float
    ) -> float:
        v_diff: float = this_speed - next_speed
        s = distance_to_next_vehicle

        s_star = self.params.s0 + this_speed * self.params.tau + this_speed * v_diff / (2 * np.sqrt(self.params.a * self.params.b))
        return self.params.a * (1 - (this_speed / self.params.v0) ** self.params.delta - (s_star / s) ** 2)

class NonConvexOptLogic:
    def __init__(self, c, idm_params):
        self.c = c
        self.p = idm_params
        self.plan = None
        self.plan_index = None

    def optimize(self, vehs):
        c = self.c
        p = self.p

        n_veh, L, N, dt, u_max = c.n_veh, c.circumference, c.n_opt, c.sim_step, c.u_max
        L_veh = vehs[0].length
        a, b, s0, T, v0, delta = p.a, p.b, p.s0, p.tau, p.v0, p.delta

        vehs = vehs[::-1]
        np.set_printoptions(linewidth=100)
        # the controlled vehicle is now the first vehicle, positions are sorted from largest to smallest
        print(f'Current positions {[veh.pos for veh in vehs]}')
        v_init = [veh.speed for veh in vehs]
        print(f'Current speeds {v_init}')
        # spacing
        s_init = [vehs[-1].pos - vehs[0].pos - L_veh] + [veh_im1.pos - veh_i.pos - L_veh for veh_im1, veh_i in zip(vehs[:-1], vehs[1:])]
        s_init = [s + L if s < 0 else s for s in s_init] # handle wrap
        print(f'Current spacings {s_init}')

        # can still follow current plan
        if self.plan is not None:
            accel = self.plan[self.plan_index]
            self.plan_index = self.plan_index + 1
            if self.plan_index == c.n_opt:
                self.plan = None
            return accel

        print(f'Optimizing trajectory for {c.n_opt} steps')

        ## solve for equilibrium conditions (when all vehicles have same spacing and going at optimal speed)
        import scipy.optimize
        sf = L / n_veh - L_veh # equilibrium space
        accel_fn = lambda v: a * (1 - (v / v0) ** delta - ((s0 + v * T) / sf) ** 2)
        sol = scipy.optimize.root(accel_fn, 0)
        vf = sol.x.item() # equilibrium speed
        sstarf = s0 + vf * T
        print('Equilibrium speed', vf)

        # nonconvex optimization
        from pydrake.all import MathematicalProgram, SnoptSolver, eq, le, ge

        # get guesses for solutions
        v_guess = [np.mean(v_init)]
        for _ in range(N):
            v_guess.append(dt * accel_fn(v_guess[-1]))
        s_guess = [sf] * (N + 1)

        prog = MathematicalProgram()
        v = prog.NewContinuousVariables(N + 1, n_veh, 'v') # speed
        s = prog.NewContinuousVariables(N + 1, n_veh, 's') # spacing
        flat = lambda x: x.reshape(-1)

        # Guess
        prog.SetInitialGuess(s, np.stack([s_guess] * n_veh, axis=1))
        prog.SetInitialGuess(v, np.stack([v_guess] * n_veh, axis=1))

        # initial conditions constraint
        prog.AddLinearConstraint(eq(v[0], v_init))
        prog.AddLinearConstraint(eq(s[0], s_init))

        # velocity constraint
        prog.AddLinearConstraint(ge(flat(v[1:]), 0))
        prog.AddLinearConstraint(le(flat(v[1:]), vf + 1)) # extra constraint to help solver

        # spacing constraint
        prog.AddLinearConstraint(ge(flat(s[1:]), s0))
        prog.AddLinearConstraint(le(flat(s[1:]), sf * 2)) # extra constraint to help solver
        prog.AddLinearConstraint(eq(flat(s[1:].sum(axis=1)), L - L_veh * n_veh))

        # spacing update constraint
        s_n = s[:-1, 1:]       # s_i[n]
        s_np1 = s[1:, 1:]      # s_i[n + 1]
        v_n = v[:-1, 1:]       # v_i[n]
        v_np1 = v[1:, 1:]      # v_i[n + 1]
        v_n_im1 = v[:-1, :-1]  # v_{i - 1}[n]
        v_np1_im1 = v[1:, :-1] # v_{i - 1}[n + 1]
        prog.AddLinearConstraint(eq(
            flat(s_np1 - s_n), flat(0.5 * dt * (v_n_im1 + v_np1_im1 - v_n - v_np1))))
        # handle position wrap for vehicle 1
        prog.AddLinearConstraint(eq(s[1:, 0] - s[:-1, 0], 0.5 * dt * (v[:-1, -1] + v[1:, -1] - v[:-1, 0] - v[1:, 0])))

        # vehicle 0's action constraint
        prog.AddLinearConstraint(ge(v[1:, 0], v[:-1, 0] - u_max * dt))
        prog.AddLinearConstraint(le(v[1:, 0], v[:-1, 0] + u_max * dt))

        # idm constraint
        prog.AddConstraint(eq(
            (v_np1 - v_n - dt * a * (1 - (v_n / v0) ** delta)) * s_n ** 2,
            -dt * a * (s0 + v_n * T + v_n * (v_n - v_n_im1) / (2 * np.sqrt(a * b))) ** 2))

        prog.AddCost(((v - vf) ** 2).mean() + ((s - sf) ** 2).mean())

        solver = SnoptSolver()
        result = solver.Solve(prog)

        assert result.is_success()

        v_desired = result.GetSolution(v)
        print('Planned speeds')
        print(v_desired)
        print('Planned spacings')
        print(result.GetSolution(s))
        a_desired = (v_desired[1:, 0] - v_desired[:-1, 0]) / dt # we're optimizing the velocity of the 0th vehicle
        self.plan = a_desired
        self.plan_index = 1
        return self.plan[0]

class PID(ControlLogic):
    def __init__(self, target_distance: float, Kp_plus: float, Kp_minus: float, Ki: float = 0.0) -> None:
        super().__init__()
        self.target_distance = target_distance
        self.error_accum: collections.deque = collections.deque(maxlen=50)
        self.error_accum.append(0.0)
        self.Kp_plus = Kp_plus
        self.Kp_minus = Kp_minus
        self.Ki = Ki

    def step_logic(self,
        distance_to_next_vehicle: float,
        this_speed: float,
        next_speed: float
    ) -> float:
        error: float = distance_to_next_vehicle - self.target_distance
        self.error_accum.append(error)
        print(f"error = {error}")
        print(f"error_accum_sum = {sum(self.error_accum)}")
        # ~ if self.steps % 75 == 0:
            # ~ self.error_accum.clear()
        p_term: float
        if error > 0:
            p_term = self.Kp_plus * error
        else:
            p_term = self.Kp_minus * error
        return p_term + self.Ki * sum(self.error_accum)

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
        self.idm_params: IDMParams = IDMParams(
            a=v_params['accel'],
            b=v_params['decel'],
            s0=v_params['minGap'],
            tau=v_params['tau'],
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

    def init_vehicles(self, highlights: Set[int], custom_controllers: Dict[int, ControlLogic]) -> None:
        """
        Creates vehicles numbered from 0 to (n_veh-1).
        :param highlights: Vehicle numbers to highlight
        :param custom_controllers: Vehicles that should use a custom controller.
        """
        c = self.c

        curr: int = 0
        interval = c.circumference / c.n_veh
        self.veh_ids: List[str] = []
        # Map of vehicles to their controllers
        self.veh_controllers: Dict[str, ControlLogic] = {}
        # Set of vehicles to highlight
        self.veh_highlights: Set[str] = set()
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
                    departPos=str(max(0, np.random.normal(curr * interval - offset, 1))),
                    departSpeed='0')

                # disable sumo checks
                self.tc.vehicle.setSpeedMode(veh_id, SPEED_MODE.aggressive)

                curr += 1
                self.veh_ids.append(veh_id)
                if curr in highlights:
                    self.veh_highlights.add(veh_id)

                if curr in custom_controllers:
                    self.veh_controllers[veh_id] = custom_controllers[curr]
                else:
                    self.veh_controllers[veh_id] = CustomIDM(self.idm_params)
        assert curr == c.n_veh

        # TODO: is this initial stepping necessary?
        # Seems like the simulation performs fine without it.
        self.tc.simulationStep()
        self.steps += 1

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
                # Find vehicle ahead of this one
                next_veh = vehs[(veh.i + 1) % len(vehs)]

                # Calculate the distance to the next vehicle
                distance_to_next_vehicle: float = next_veh.pos - veh.pos - veh.length
                if distance_to_next_vehicle < 0:
                    distance_to_next_vehicle += c.circumference

                # Compute output acceleration
                if isinstance(self.veh_controllers[veh.id], ControlLogic):
                    veh.accel = self.veh_controllers[veh.id].step(
                        distance_to_next_vehicle=distance_to_next_vehicle,
                        this_speed=veh.speed,
                        next_speed=next_veh.speed
                    )
                else:
                    veh.accel = self.veh_controllers[veh.id].optimize(vehs)

                # Highlight this vehicle if needed
                if veh.id in self.veh_highlights:
                    self.tc.vehicle.setColor(veh.id, color=(255, 111, 0, 200))

                # verified that setSpeed sets speed immediately, slowDown linearly decelerates vehicle over duration
                vehicle.slowDown(veh.id, max(0, veh.speed + c.sim_step * veh.accel), duration=c.sim_step)

        speeds = [veh.speed for veh in vehs]
        print('step', self.steps)
        print('speeds', ' '.join(('%.3g' % speed) for speed in sorted(speeds)))
        print('mean %.5g min %.5g max %.5g' % (np.mean(speeds), np.min(speeds), np.max(speeds)))
        print()
        self.tc.simulationStep()
        # handle the fact that SUMO doesn't move the vehicles exactly as we predict
        if c.get('custom_move', False):
            for veh in vehs:
                new_speed = veh.speed + veh.accel * c.sim_step
                pos = veh.pos + (veh.speed + new_speed) / 2 * c.sim_step
                if pos > c.circumference:
                    pos -= c.circumference
                if pos >= c.circumference / 2:
                    lane_pos = pos - c.circumference / 2
                    lane = 'left_0'
                else:
                    lane_pos = pos
                    lane = 'right_0'
                vehicle.moveTo(veh.id, lane, lane_pos)
                vehicle.setSpeed(veh.id, max(0, new_speed))
        self.steps += 1

def baseline() -> None:
    """
    Do a baseline run (all IDM).
    """
    c = Namespace(
        res=Path('tmp'),
        horizon=3000,

        n_veh=10,
        circumference=250,
        sim_step=0.25,
        render=True,

        custom_update=True,
        custom_move=True,
    ).var(**from_args())
    env = RingEnv(c)
    env.init_vehicles(
        highlights={},
        custom_controllers={}
    )
    for t in range(c.horizon):
        env.step()

def nonconvex_opt() -> None:
    c = Namespace(
        res=Path('tmp'),
        horizon=3000,

        n_veh=8,
        circumference=80,
        sim_step=1,
        render=True,

        custom_update=True,
        custom_move=True,
        n_opt=5,
        dt=1,
        u_max=1,
    ).var(**from_args())
    env = RingEnv(c)
    env.init_vehicles(
        highlights={c.n_veh},
        custom_controllers={c.n_veh: NonConvexOptLogic(c, env.idm_params)} # Always control the last vehicle
    )
    for t in range(c.horizon):
        env.step()

def pid_short_leash() -> None:
    """
    Run a PID loop, following closely.
    """
    c = Namespace(
        res=Path('tmp'),
        horizon=3000,

        n_veh=10,
        circumference=250,
        sim_step=0.25,
        render=True,

        custom_update=True
    ).var(**from_args())
    env = RingEnv(c)
    env.init_vehicles(
        highlights={5},
        # Close following requires very high braking forces
        custom_controllers={5: PID(9.0,
            Kp_plus=1.8, Kp_minus=26.5,
            # In close following, for some reason, the integral term
            # just builds up and up and causes wild instability later.
            Ki=0.0
        )}
    )
    for t in range(c.horizon):
        env.step()

def pid_long_leash() -> None:
    """
    Run a PID loop, following with longer distance.
    """
    c = Namespace(
        res=Path('tmp'),
        horizon=3000,

        n_veh=10,
        circumference=250,
        sim_step=0.25,
        render=True,

        custom_update=True
    ).var(**from_args())
    env = RingEnv(c)
    env.init_vehicles(
        highlights={5},
        custom_controllers={5: PID(90.0,
            Kp_plus=6.0, Kp_minus=12.0,
            Ki=0.1
        )}
    )
    for t in range(c.horizon):
        env.step()

def silly_controller() -> None:
    """
    Run a silly controller.
    """
    c = Namespace(
        res=Path('tmp'),
        horizon=3000,

        n_veh=10,
        circumference=250,
        sim_step=0.25,
        render=True,

        custom_update=True
    ).var(**from_args())
    env = RingEnv(c)
    class Silly(ControlLogic):
        def step_logic(self,
            distance_to_next_vehicle: float,
            this_speed: float,
            next_speed: float
        ) -> float:
            print(f"distance_to_next_vehicle = {distance_to_next_vehicle}")
            if distance_to_next_vehicle < 60:
                if self.steps % 40 < 20:
                    return 0.2
                else:
                    return -0.212
            else:
                return 0.6
    env.init_vehicles(
        highlights={5},
        custom_controllers={5: Silly()}
    )
    for t in range(c.horizon):
        env.step()

if __name__ == '__main__':
    # baseline()
    # pid_short_leash()
    # pid_long_leash()
    # silly_controller()
    nonconvex_opt()
