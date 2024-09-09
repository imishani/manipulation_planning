#!/usr/bin/env python3

import os
import glob
import rospy
from manipulation_planning.srv import TriggerNN, TriggerNNResponse
import pickle
from math import ceil
from pathlib import Path
import numpy as np

import matplotlib.pyplot as plt
import torch


from motion_planning_baselines.mp_baselines.planners.costs.cost_functions import CostCollision, CostComposite, CostGPTrajectory
from mpd.models import TemporalUnet, UNET_DIM_MULTS
from mpd.models.diffusion_models.guides import GuideManagerTrajectoriesWithVelocity
from mpd.models.diffusion_models.sample_functions import guide_gradient_steps, ddpm_sample_fn
from mpd.trainer import get_dataset, get_model
from mpd.utils.loading import load_params_from_yaml
from torch_robotics.torch_utils.seed import fix_random_seed
from torch_robotics.torch_utils.torch_timer import TimerCUDA
from torch_robotics.torch_utils.torch_utils import get_torch_device, freeze_torch_model_params
from torch_robotics.trajectory.utils import interpolate_traj_via_points


TRAINED_MODELS_DIR = "/home/spencer/nn_model/data_trained_models"

def init_model():
    ########################################################################################################################
    # Experiment configuration
    model_id = 'EnvSpheres3D-RobotPanda'
    planner_alg = 'mpd'

    use_guide_on_extra_objects_only: bool = False
    start_guide_steps_fraction: float = 0.25
    n_guide_steps: int = 5
    weight_grad_cost_collision: float = 1e-2
    weight_grad_cost_smoothness: float = 1e-7
    factor_num_interpolated_points_for_collision: float = 1.5
    trajectory_duration: float = 5.0  # currently fixed
    device: str = 'cuda'

    results_dir: str = 'logs'
    seed: int = 30

    # Check if CUDA is available
    cuda_available = torch.cuda.is_available()
    print(f"CUDA is available: {cuda_available}")
    fix_random_seed(seed)
    device = get_torch_device(device)
    tensor_args = {'device': device, 'dtype': torch.float32}

    print(f'##########################################################################################################')
    print(f'Model -- {model_id}')
    print(f'Algorithm -- {planner_alg}')
    run_prior_only = False
    run_prior_then_guidance = False
    
    model_dir = os.path.join(TRAINED_MODELS_DIR, model_id)
    results_dir = os.path.join(model_dir, 'results_inference', str(seed))
    print(results_dir)
    os.makedirs(results_dir, exist_ok=True)

    args = load_params_from_yaml(os.path.join(model_dir, "args.yaml"))

    # Load dataset with env, robot, task
    train_subset, train_dataloader, val_subset, val_dataloader = get_dataset(
        dataset_class='TrajectoryDataset',
        use_extra_objects=True,
        obstacle_cutoff_margin=0.05,
        **args,
        tensor_args=tensor_args
    )
    dataset = train_subset.dataset
    n_support_points = dataset.n_support_points
    env = dataset.env
    robot = dataset.robot
    task = dataset.task

    dt = trajectory_duration / n_support_points  # time interval for finite differences

    # set robot's dt
    robot.dt = dt
    

    ########################################################################################################################
    # Load prior model
    diffusion_configs = dict(
        variance_schedule=args['variance_schedule'],
        n_diffusion_steps=args['n_diffusion_steps'],
        predict_epsilon=args['predict_epsilon'],
    )
    unet_configs = dict(
        state_dim=dataset.state_dim,
        n_support_points=dataset.n_support_points,
        unet_input_dim=args['unet_input_dim'],
        dim_mults=UNET_DIM_MULTS[args['unet_dim_mults_option']],
    )
    diffusion_model = get_model(
        model_class=args['diffusion_model_class'],
        model=TemporalUnet(**unet_configs),
        tensor_args=tensor_args,
        **diffusion_configs,
        **unet_configs
    )
    diffusion_model.load_state_dict(
        torch.load(os.path.join(model_dir, 'checkpoints', 'ema_model_current_state_dict.pth' if args['use_ema'] else 'model_current_state_dict.pth'),
        map_location=tensor_args['device'])
    )
    diffusion_model.eval()
    model = diffusion_model

    freeze_torch_model_params(model)
    model = torch.compile(model)
    model.warmup(horizon=n_support_points, device=device)

    ########
    # Set up the planning costs

    # Cost collisions
    cost_collision_l = []
    weights_grad_cost_l = []  # for guidance, the weights_cost_l are the gradient multipliers (after gradient clipping)
    if use_guide_on_extra_objects_only:
        collision_fields = task.get_collision_fields_extra_objects()
    else:
        collision_fields = task.get_collision_fields()

    for collision_field in collision_fields:
        cost_collision_l.append(
            CostCollision(
                robot, n_support_points,
                field=collision_field,
                sigma_coll=1.0,
                tensor_args=tensor_args
            )
        )
        weights_grad_cost_l.append(weight_grad_cost_collision)

    # Cost smoothness
    cost_smoothness_l = [
        CostGPTrajectory(
            robot, n_support_points, dt, sigma_gp=1.0,
            tensor_args=tensor_args
        )
    ]
    weights_grad_cost_l.append(weight_grad_cost_smoothness)

    ####### Cost composition
    cost_func_list = [
        *cost_collision_l,
        *cost_smoothness_l
    ]

    cost_composite = CostComposite(
        robot, n_support_points, cost_func_list,
        weights_cost_l=weights_grad_cost_l,
        tensor_args=tensor_args
    )

    ########
    # Guiding manager
    guide = GuideManagerTrajectoriesWithVelocity(
            dataset,
            cost_composite,
            clip_grad=True,
            interpolate_trajectories_for_collision=True,
            num_interpolated_points=ceil(n_support_points * factor_num_interpolated_points_for_collision),
            tensor_args=tensor_args,
    )

    t_start_guide = ceil(start_guide_steps_fraction * model.n_diffusion_steps)
    sample_fn_kwargs = dict(
        guide=None if run_prior_then_guidance or run_prior_only else guide,
        n_guide_steps=n_guide_steps,
        t_start_guide=t_start_guide,
        noise_std_extra_schedule_fn=lambda x: 0.5,
    )

    return model, dataset, guide, sample_fn_kwargs, run_prior_then_guidance, t_start_guide

def read_input():
    ########################################################################################################################
    # Random initial and final positions
    device: str = 'cuda'
    device = get_torch_device(device)
    start_state_pos, goal_state_pos = None, None
    vec1, vec2 = [], []
    input_file_path = "/home/spencer/repos/manipulation_ws/src/manipulation_planning/data/experiences_from_model/panda_arm/input"
    print("Reading from: " , input_file_path)

    with open(input_file_path, 'r') as file:
        # Read the first line, split into numbers, and convert to floats
        line1 = file.readline().strip().split()
        vec1 = [float(x) for x in line1]

        # Read the second line, split into numbers, and convert to floats
        line2 = file.readline().strip().split()
        vec2 = [float(x) for x in line2]
    
    print(vec1)
    print(vec2)

    start_state_pos = torch.tensor(vec1).to(device)
    goal_state_pos = torch.tensor(vec2).to(device)

    print(f'start_state_pos: {start_state_pos}')
    print(f'goal_state_pos: {goal_state_pos}')
    
    return start_state_pos, goal_state_pos

def handle_nn_request(req):
    rospy.loginfo("Received signal request")

    env = dataset.env
    robot = dataset.robot
    task = dataset.task
    n_support_points = dataset.n_support_points
    n_samples: int = 50
    n_diffusion_steps_without_noise: int = 5
    context = None
    start_state_pos, goal_state_pos = read_input()
    ########
    # normalize start and goal positions
    hard_conds = dataset.get_hard_conditions(torch.vstack((start_state_pos, goal_state_pos)), normalize=True)
    context = None

    ########
    # Sample trajectories with the diffusion/cvae model
    with TimerCUDA() as timer_model_sampling:
        trajs_normalized_iters = model.run_inference(
            context, hard_conds,
            n_samples=n_samples, horizon=n_support_points,
            return_chain=True,
            sample_fn=ddpm_sample_fn,
            **sample_fn_kwargs,
            n_diffusion_steps_without_noise=n_diffusion_steps_without_noise,
            # ddim=True
        )
    print(f't_model_sampling: {timer_model_sampling.elapsed:.3f} sec')
    t_total = timer_model_sampling.elapsed


    print(f'\n----------------METRICS----------------')
    print(f't_total: {t_total:.3f} sec')

    # unnormalize trajectory samples from the models
    trajs_iters = dataset.unnormalize_trajectories(trajs_normalized_iters)

    trajs_final = trajs_iters[-1]
    trajs_final_coll, trajs_final_coll_idxs, trajs_final_free, trajs_final_free_idxs, _ = task.get_trajs_collision_and_free(trajs_final, return_indices=True)



    trajs_out = robot.get_position(trajs_final_coll).movedim(1, 0)
    trajs_out = interpolate_traj_via_points(trajs_out.movedim(0, 1), 2).movedim(1, 0)
    trajs_out = trajs_out.cpu()

    # Directory path
    directory = '/home/spencer/repos/manipulation_ws/src/manipulation_planning/data/experiences_from_model/panda_arm'

    pattern = os.path.join(directory, 'path_*.csv')

    # Find all files matching the pattern
    files_to_delete = glob.glob(pattern)

    # Delete each file
    for file_path in files_to_delete:
        try:
            os.remove(file_path)
        except Exception as e:
            print(f"Error deleting {file_path}: {e}")


    # Iterate through each trajectory and save as a CSV
    for i in range(trajs_out.size(1)):  # 50 trajectories
        if i == 10:
            break
        # Extract the i-th trajectory, shape [n, 7]
        trajectory = trajs_out[:, i, :].numpy()  # Convert to NumPy array

        # Create a file name for the trajectory
        file_path = os.path.join(directory, f'path_{i}.csv')

       # Open the file and write the formatted data
        with open(file_path, 'w') as f:
            # Write the header: Experience,length_of_trajectory,7
            length_of_trajectory = trajectory.shape[0]  # Number of steps (n)
            f.write(f"Experience,{length_of_trajectory + 1},7\n")

            step_str = ','.join(f'{i:.6f}' for i in start_state_pos.cpu().numpy())
            f.write(f"{step_str},\n")  # Add a trailing comma as required
            
            # Write each step in the trajectory
            for step in trajectory:
                step_str = ','.join(f'{x:.6f}' for x in step)  # Format each value with 6 decimals
                f.write(f"{step_str},\n")  # Add a trailing comma as required

    print(f'Saved trajectory {i} to {file_path}')


    return TriggerNNResponse(success = True)


def nn_inference_server():
    rospy.init_node("nn_inference_server")
    global model, dataset, guide, sample_fn_kwargs, run_prior_then_guidance, t_start_guide
    model, dataset, guide, sample_fn_kwargs, run_prior_then_guidance, t_start_guide = init_model()

    service = rospy.Service('nn_trigger', TriggerNN, handle_nn_request)

    rospy.loginfo("NN inference server ready")
    rospy.spin()

if __name__ == "__main__":
    nn_inference_server()


