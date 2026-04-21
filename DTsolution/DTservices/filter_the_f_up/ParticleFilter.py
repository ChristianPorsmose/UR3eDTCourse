import numpy as np
from numpy.typing import NDArray
from typing import Optional

Array = NDArray[np.float64]


class ParticleFilter:
    def __init__(self, num_particles: int, dim: int) -> None:
        self.num_particles = num_particles
        self.dim = dim 

        self.state_dim = 2 * dim

        self.particles: Array = np.random.normal(
            0, 1, (num_particles, self.state_dim)
        )
        self.weights: Array = np.ones(num_particles) / num_particles

        self.old_time: Optional[float] = None

        self.process_noise_std = 0.05
        self.measurement_noise_std = 0.1

        self.initialized = False

    @property
    def is_initialized(self) -> bool:
        return self.initialized


    def initialize(self, state_pos: Array, state_vel: Array, timestamp: float) -> None:
        assert state_pos.shape == (self.dim,)
        assert state_vel.shape == (self.dim,)

        self.particles = np.hstack([
            np.random.normal(state_pos, 0.1, (self.num_particles, self.dim)),
            np.random.normal(state_vel, 0.1, (self.num_particles, self.dim)),
        ])

        self.weights.fill(1.0 / self.num_particles)
        self.old_time = timestamp

        self.initialized = True


    def predict(self, dt: float) -> None:
        pos = self.particles[:, :self.dim]
        vel = self.particles[:, self.dim:]

        noise_pos = np.random.normal(
            0, self.process_noise_std * np.sqrt(dt), pos.shape
        )
        noise_vel = np.random.normal(
            0, self.process_noise_std * np.sqrt(dt), vel.shape
        )

        pos = pos + vel * dt + noise_pos
        vel = vel + noise_vel

        self.particles[:, :self.dim] = pos
        self.particles[:, self.dim:] = vel

    def update(self, measurement_pos: Array) -> None:
        pos = self.particles[:, :self.dim]

        error = pos - measurement_pos
        dist2 = np.sum(error**2, axis=1)

        self.weights = np.exp(
            -0.5 * dist2 / (self.measurement_noise_std**2)
        )

        self.weights += 1e-300
        self.weights /= np.sum(self.weights)

    def resample(self) -> None:
        idx = np.random.choice(
            self.num_particles,
            size=self.num_particles,
            p=self.weights
        )
        self.particles = self.particles[idx]
        self.weights.fill(1.0 / self.num_particles)

    def estimate(self) -> tuple[Array, Array]:
        pos = np.mean(self.particles[:, :self.dim], axis=0)
        vel = np.mean(self.particles[:, self.dim:], axis=0)
        return pos, vel