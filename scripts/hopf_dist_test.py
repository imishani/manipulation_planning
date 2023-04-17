import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import healpy as hp
import pickle


def hopf_dist_samples(
        n_samples: int):
    """
    Generate hopf distribution samples
    by sampling from the uniform distribution
    the range of each angle:
    theta: [0, pi]
    phi: [0, 2pi]
    psi: [-pi, pi]

    input: n_samples: number of samples to generate
    output: np.array of shape (n_samples, 3)
    """
    theta = np.random.uniform(0, np.pi, n_samples)
    phi = np.random.uniform(0, 2 * np.pi, n_samples)
    psi = np.random.uniform(0, 2 * np.pi, n_samples)
    return np.stack([theta, phi, psi], axis=1)


def hopf_to_quaternion(hopf: np.array):
    """
    Convert hopf distribution samples to quaternions
    input: hopf: np.array of shape (n_samples, 3)
    output: np.array of shape (n_samples, 4)
    """
    theta = hopf[:, 0]
    phi = hopf[:, 1]
    psi = hopf[:, 2]

    w = np.cos(theta / 2) * np.cos(psi / 2)
    x = np.cos(theta / 2) * np.sin(psi / 2)
    y = np.sin(theta / 2) * np.cos(phi + psi / 2)
    z = np.sin(theta / 2) * np.sin(phi + psi / 2)

    return np.stack([w, x, y, z], axis=1)


def quaternion_to_hopf(quaternion: np.array):
    """
    Convert quaternions to hopf distribution samples
    input: quaternion: np.array of shape (n_samples, 4)
    output: np.array of shape (n_samples, 3)
    """
    w = quaternion[:, 0]
    x = quaternion[:, 1]
    y = quaternion[:, 2]
    z = quaternion[:, 3]

    # According to drake
    # make sure that w >= 0 and if not, negate the quaternion
    if (w < 0).any():
        w = np.where(w < 0, -w, w)
        x = np.where(w < 0, -x, x)
        y = np.where(w < 0, -y, y)
        z = np.where(w < 0, -z, z)
    psi = 2 * np.arctan2(x, w)
    phi_unwrapped = np.arctan2(z, y) - psi / 2
    phi = np.where(phi_unwrapped < 0, phi_unwrapped + 2 * np.pi, phi_unwrapped)
    theta = 2 * np.arctan2(np.sqrt(y ** 2 + z ** 2), np.sqrt(w ** 2 + x ** 2))

    return np.stack([theta, phi, psi], axis=1)


def hopf_dist_test():
    """
    Test hopf distribution
    """
    n_samples = 10000
    hopf_samples = hopf_dist_samples(n_samples)
    quaternion_samples = hopf_to_quaternion(hopf_samples)
    hopf_samples2 = quaternion_to_hopf(quaternion_samples)
    # Plot the samples in 3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(hopf_samples[:, 0], hopf_samples[:, 1], hopf_samples[:, 2], alpha=0.1)
    ax.set_xlabel('theta')
    ax.set_ylabel('phi')
    ax.set_zlabel('psi')
    plt.show()


# generate a uniform distribution of SO(3) samples using hopf coordinates and healpix sampling
def hopf_dist_samples_healpix(
        n_samples: int,
        nside: int = 32):
    """
    Generate hopf distribution samples
    by sampling from S1 and S2 using healpix sampling
    the range of each angle:
    theta: [0, pi]
    phi: [0, 2pi]
    psi: [-pi, pi]

    input: n_samples: number of samples to generate
    output: np.array of shape (n_samples, 3)
    """
    # https://en.wikipedia.org/wiki/Healpix
    print(f"The resolution of the healpix sampling is {hp.nside2resol(nside)}")
    theta, phi = hp.pix2ang(nside, np.arange(hp.nside2npix(nside)))
    psi = np.linspace(-np.pi, np.pi, theta.shape[0])
    # take each theta and phi and samples and use the to generate 3d points on the unit sphere
    x = np.sin(theta) * np.cos(phi)
    y = np.sin(theta) * np.sin(phi)
    z = np.cos(theta)
    # plot the samples in 3D
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z, alpha=0.4)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()

    # Check if all points are on the unit sphere
    assert np.allclose(np.sqrt(x ** 2 + y ** 2 + z ** 2), 1)
    print(f"Number of samples: {len(x)}")
    return np.stack([theta, phi, psi], axis=1)


def saveSamples(samples, filename):
    """
    Save the samples as a kd-tree

    :param samples:
    :param filename:
    :return:
    """
    # Save to file which can be loaded later in c++
    np.savetxt(filename, samples, delimiter=",")


def discretizing_test(samples_: np.array):
    # generate a random quaternion
    q = np.random.uniform(-1, 1, 4)
    # make sure to take one side of the sphere
    if q[0] < 0:
        q = -q
    # normalize the quaternion
    q /= np.linalg.norm(q)
    print(f"q: {q}")
    # convert to hopf coordinates
    hopf = quaternion_to_hopf(q[None, :])[0]
    print(f"hopf: {hopf}")
    # check the quaternion
    q_test = hopf_to_quaternion(hopf[None, :])[0]
    print(f"q_test: {q_test}")

    # look for the closest sample
    idx = np.argmin(np.linalg.norm(samples_[:, :2] - hopf[:2], axis=1))
    idx2 = np.argmin(np.linalg.norm((samples_[:, 2] - hopf[2]).reshape(-1, 1), axis=1))
    print(f"idx: {idx}, idx2: {idx2}")
    print(f"sample: {samples_[idx, :2]} {samples_[idx2, 2]}")

    # convert the closest sample to quaternion
    # take the first two elements from idx and the third from idx2
    hopf_closest = np.array([samples_[idx, 0], samples_[idx, 1], samples_[idx2, 2]])
    q2 = hopf_to_quaternion(hopf_closest[None, :])[0]
    print(f"q2: {q2}")
    # make sure the two quaternions are the same
    print(f"q*q_dis: {1 - q.dot(q2)}")
    vec = hp.ang2vec(hopf[0], hopf[1])
    ip_ring = hp.query_disc(nside=8, vec=vec, radius=np.deg2rad(5), inclusive=False)
    test = hp.query_strip(nside=8, theta1=hopf[0], theta2=hopf[1], inclusive=True)
    i = 0

# main
if __name__ == '__main__':
    # hopf_dist_test()
    samples = hopf_dist_samples_healpix(10, 8)
    # saveSamples(samples, "../config/hopf_dist_samples_healpix.txt")
    discretizing_test(samples)
