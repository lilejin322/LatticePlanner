"""Vehicle motion model aligned with modules/common/vehicle_model/vehicle_model.cc."""

import math

from protoclass.vehicle_state import VehicleState

kRearCenteredKinematicBicycleModelDt: float = 0.06
"""Matches modules/common/vehicle_model/conf/vehicle_model_config.pb.txt's
rc_kinematic_bicycle_model.dt (the only model type in this project's scope)."""


class VehicleModel:
    """
    VehicleModel class
    """

    @staticmethod
    def RearCenteredKinematicBicycleModel(
        predicted_time_horizon: float, cur_vehicle_state: VehicleState
    ) -> VehicleState:
        """
        Kinematic bicycle model centered at rear axis center by Euler forward
        discretization. Assumes constant control command and constant z position.

        :param float predicted_time_horizon: Time horizon to predict forward
        :param VehicleState cur_vehicle_state: Current vehicle state
        :returns: Predicted vehicle state
        :rtype: VehicleState
        """

        assert predicted_time_horizon > 0.0, "predicted_time_horizon must be positive"
        dt: float = kRearCenteredKinematicBicycleModelDt
        cur_x: float = cur_vehicle_state.x
        cur_y: float = cur_vehicle_state.y
        cur_z: float = cur_vehicle_state.z or 0.0
        cur_phi: float = cur_vehicle_state.heading
        cur_v: float = cur_vehicle_state.linear_velocity or 0.0
        cur_a: float = cur_vehicle_state.linear_acceleration or 0.0
        cur_kappa: float = cur_vehicle_state.kappa or 0.0
        next_x: float = cur_x
        next_y: float = cur_y
        next_phi: float = cur_phi
        next_v: float = cur_v

        if dt >= predicted_time_horizon:
            dt = predicted_time_horizon

        countdown_time: float = predicted_time_horizon
        finish_flag: bool = False
        kepsilon: float = 1e-8
        while countdown_time > kepsilon and not finish_flag:
            countdown_time -= dt
            if countdown_time < kepsilon:
                dt = countdown_time + dt
                finish_flag = True
            intermediate_phi: float = cur_phi + 0.5 * dt * cur_v * cur_kappa
            next_phi = cur_phi + dt * (cur_v + 0.5 * dt * cur_a) * cur_kappa
            next_x = cur_x + dt * (cur_v + 0.5 * dt * cur_a) * math.cos(intermediate_phi)
            next_y = cur_y + dt * (cur_v + 0.5 * dt * cur_a) * math.sin(intermediate_phi)
            next_v = cur_v + dt * cur_a
            cur_x = next_x
            cur_y = next_y
            cur_phi = next_phi
            cur_v = next_v

        return VehicleState(
            x=next_x,
            y=next_y,
            z=cur_z,
            heading=next_phi,
            kappa=cur_kappa,
            linear_velocity=next_v,
            linear_acceleration=cur_vehicle_state.linear_acceleration,
        )

    @staticmethod
    def Predict(predicted_time_horizon: float, cur_vehicle_state: VehicleState) -> VehicleState:
        """
        Predict a future vehicle state. Only the rear-centered kinematic
        bicycle model is supported, matching this project's translation scope.

        :param float predicted_time_horizon: Time horizon to predict forward
        :param VehicleState cur_vehicle_state: Current vehicle state
        :returns: Predicted vehicle state
        :rtype: VehicleState
        """

        return VehicleModel.RearCenteredKinematicBicycleModel(predicted_time_horizon, cur_vehicle_state)
