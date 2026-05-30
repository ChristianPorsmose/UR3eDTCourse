import numpy as np
import pytest
from dt_solution.dt_services.particle_filter.particle_filter import ParticleFilter

def make_filter(num_particles=500, dim=3) -> ParticleFilter:
    return ParticleFilter(num_particles=num_particles, dim=dim)


def initialized_filter(pos=None, vel=None, dim=3, num_particles=500) -> ParticleFilter:
    pf = make_filter(num_particles=num_particles, dim=dim)
    pos = pos if pos is not None else np.zeros(dim)
    vel = vel if vel is not None else np.zeros(dim)
    pf.initialize(pos, vel, timestamp=0.0)
    return pf

class TestFullCycle:
    def test_filter_tracks_stationary_target(self):
        """After several update/resample cycles the filter should converge."""
        np.random.seed(99)
        true_pos = np.array([3.0, -1.0, 2.0])
        true_vel = np.zeros(3)

        pf = initialized_filter(pos=true_pos, vel=true_vel, num_particles=1000)

        for _ in range(20):
            pf.predict(dt=0.1)
            measurement = true_pos + np.random.normal(0, pf.measurement_noise_std, 3)
            pf.update(measurement)
            pf.resample()

        pos_est, _ = pf.estimate()
        np.testing.assert_allclose(pos_est, true_pos, atol=0.3)

    def test_filter_tracks_moving_target(self):
        """Filter should track a target moving at constant velocity."""
        np.random.seed(0)
        true_pos = np.array([0.0, 0.0])
        true_vel = np.array([1.0, 0.5])
        dt = 0.05

        pf = initialized_filter(pos=true_pos, vel=true_vel, dim=2, num_particles=1000)

        for _ in range(40):
            true_pos = true_pos + true_vel * dt
            pf.predict(dt=dt)
            measurement = true_pos + np.random.normal(0, pf.measurement_noise_std, 2)
            pf.update(measurement)
            pf.resample()

        pos_est, vel_est = pf.estimate()
        np.testing.assert_allclose(pos_est, true_pos, atol=0.5)
        np.testing.assert_allclose(vel_est, true_vel, atol=0.5)
