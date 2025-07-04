import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from common_msgs.msg import TrajectoryPoints, TrajectoryPoint
from geometry_msgs.msg import Point

from .global_track import Track
from .global_trajectory import Trajectory
import time
from .elaborate_output import elaborate_output


class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('race_status_sub')
        self.params_dict = self.get_params()
        self.got_it = False

        self.centerline_sub = self.create_subscription(
            Marker,
            self.params_dict['topics']['center_line_completed'],
            self.__centerline_sub_callback,
            10)

        self.speed_profile_pub = self.create_publisher(
            TrajectoryPoints, self.params_dict['topics']['trajectory_points'], 10)

        self.track = Track(debug=self.params_dict['misc']['debug'])
        self.trajectory = Trajectory(params=self.params_dict)

        initialization_message = (
            "Global planner initializated.\n"
            "   ___ _     _         _   ___ _                        \n"
            "  / __| |___| |__ __ _| | | _ | |__ _ _ _  _ _  ___ _ _ \n"
            " | (_ | / _ | '_ / _` | | |  _| / _` | ' \| ' \/ -_| '_|\n"
            "  \___|_\___|_.__\__,_|_| |_| |_\__,_|_||_|_||_\___|_|  \n"
        )

        self.get_logger().info(initialization_message)

    def get_params(self):
        params_dict = {}
        # generated params loader for ROS
        params_dict['topics'] = {}
        self.declare_parameter('topics.race_status')
        params_dict['topics']['race_status'] = self.get_parameter(
            'topics.race_status').value
        self.declare_parameter('topics.center_line_completed')
        params_dict['topics']['center_line_completed'] = self.get_parameter(
            'topics.center_line_completed').value
        self.declare_parameter('topics.trajectory_points')
        params_dict['topics']['trajectory_points'] = self.get_parameter(
            'topics.trajectory_points').value

        params_dict['car_config'] = {}
        self.declare_parameter('car_config.ggv_file')
        params_dict['car_config']['ggv_file'] = self.get_parameter(
            'car_config.ggv_file').value
        self.declare_parameter('car_config.ax_max_machines_file')
        params_dict['car_config']['ax_max_machines_file'] = self.get_parameter(
            'car_config.ax_max_machines_file').value
        params_dict['car_config']['stepsize_opts'] = {}
        self.declare_parameter('car_config.stepsize_opts.stepsize_prep')
        params_dict['car_config']['stepsize_opts']['stepsize_prep'] = self.get_parameter(
            'car_config.stepsize_opts.stepsize_prep').value
        self.declare_parameter('car_config.stepsize_opts.stepsize_reg')
        params_dict['car_config']['stepsize_opts']['stepsize_reg'] = self.get_parameter(
            'car_config.stepsize_opts.stepsize_reg').value
        self.declare_parameter(
            'car_config.stepsize_opts.stepsize_interp_after_opt')
        params_dict['car_config']['stepsize_opts']['stepsize_interp_after_opt'] = self.get_parameter(
            'car_config.stepsize_opts.stepsize_interp_after_opt').value
        params_dict['car_config']['reg_smooth_opts'] = {}
        self.declare_parameter('car_config.reg_smooth_opts.k_reg')
        params_dict['car_config']['reg_smooth_opts']['k_reg'] = self.get_parameter(
            'car_config.reg_smooth_opts.k_reg').value
        self.declare_parameter('car_config.reg_smooth_opts.s_reg')
        params_dict['car_config']['reg_smooth_opts']['s_reg'] = self.get_parameter(
            'car_config.reg_smooth_opts.s_reg').value
        params_dict['car_config']['veh_params'] = {}
        self.declare_parameter('car_config.veh_params.v_max')
        params_dict['car_config']['veh_params']['v_max'] = self.get_parameter(
            'car_config.veh_params.v_max').value
        self.declare_parameter('car_config.veh_params.length')
        params_dict['car_config']['veh_params']['length'] = self.get_parameter(
            'car_config.veh_params.length').value
        self.declare_parameter('car_config.veh_params.width')
        params_dict['car_config']['veh_params']['width'] = self.get_parameter(
            'car_config.veh_params.width').value
        self.declare_parameter('car_config.veh_params.mass')
        params_dict['car_config']['veh_params']['mass'] = self.get_parameter(
            'car_config.veh_params.mass').value
        self.declare_parameter('car_config.veh_params.dragcoeff')
        params_dict['car_config']['veh_params']['dragcoeff'] = self.get_parameter(
            'car_config.veh_params.dragcoeff').value
        self.declare_parameter('car_config.veh_params.curvlim')
        params_dict['car_config']['veh_params']['curvlim'] = self.get_parameter(
            'car_config.veh_params.curvlim').value
        self.declare_parameter('car_config.veh_params.g')
        params_dict['car_config']['veh_params']['g'] = self.get_parameter(
            'car_config.veh_params.g').value
        params_dict['car_config']['vel_calc_opts'] = {}
        self.declare_parameter('car_config.vel_calc_opts.dyn_model_exp')
        params_dict['car_config']['vel_calc_opts']['dyn_model_exp'] = self.get_parameter(
            'car_config.vel_calc_opts.dyn_model_exp').value
        self.declare_parameter(
            'car_config.vel_calc_opts.vel_profile_conv_filt_window')
        params_dict['car_config']['vel_calc_opts']['vel_profile_conv_filt_window'] = self.get_parameter(
            'car_config.vel_calc_opts.vel_profile_conv_filt_window').value
        
        params_dict['optimization_opt'] = {}
        params_dict['optimization_opt']['optim_opts_mincurv'] = {}
        self.declare_parameter('optimization_opt.optim_opts_mincurv.width_opt')
        params_dict['optimization_opt']['optim_opts_mincurv']['width_opt'] = self.get_parameter(
            'optimization_opt.optim_opts_mincurv.width_opt').value
        self.declare_parameter(
            'optimization_opt.optim_opts_mincurv.iqp_iters_min')
        params_dict['optimization_opt']['optim_opts_mincurv']['iqp_iters_min'] = self.get_parameter(
            'optimization_opt.optim_opts_mincurv.iqp_iters_min').value
        self.declare_parameter(
            'optimization_opt.optim_opts_mincurv.iqp_curverror_allowed')
        params_dict['optimization_opt']['optim_opts_mincurv']['iqp_curverror_allowed'] = self.get_parameter(
            'optimization_opt.optim_opts_mincurv.iqp_curverror_allowed').value
        
        params_dict['misc'] = {}
        self.declare_parameter('misc.savePointsPath')
        params_dict['misc']['savePointsPath'] = self.get_parameter(
            'misc.savePointsPath').value
        self.declare_parameter('misc.fakeDistance')
        params_dict['misc']['fakeDistance'] = self.get_parameter(
            'misc.fakeDistance').value
        self.declare_parameter('misc.legacylocalTopic')
        params_dict['misc']['legacylocalTopic'] = self.get_parameter(
            'misc.legacylocalTopic').value
        self.declare_parameter('misc.debug')
        params_dict['misc']['debug'] = self.get_parameter('misc.debug').value

        params_dict['imp_opts'] = {}
        self.declare_parameter('imp_opts.min_track_width')
        params_dict['imp_opts']['min_track_width'] = self.get_parameter(
            'imp_opts.min_track_width').value
        
        params_dict['lap_time_mat_opts'] = {}
        self.declare_parameter('lap_time_mat_opts.use_lap_time_mat')
        params_dict['lap_time_mat_opts']['use_lap_time_mat'] = self.get_parameter(
            'lap_time_mat_opts.use_lap_time_mat').value
        self.declare_parameter('lap_time_mat_opts.gg_scale_range')
        params_dict['lap_time_mat_opts']['gg_scale_range'] = self.get_parameter(
            'lap_time_mat_opts.gg_scale_range').value
        self.declare_parameter('lap_time_mat_opts.gg_scale_stepsize')
        params_dict['lap_time_mat_opts']['gg_scale_stepsize'] = self.get_parameter(
            'lap_time_mat_opts.gg_scale_stepsize').value
        self.declare_parameter('lap_time_mat_opts.top_speed_range')
        params_dict['lap_time_mat_opts']['top_speed_range'] = self.get_parameter(
            'lap_time_mat_opts.top_speed_range').value
        self.declare_parameter('lap_time_mat_opts.top_speed_stepsize')
        params_dict['lap_time_mat_opts']['top_speed_stepsize'] = self.get_parameter(
            'lap_time_mat_opts.top_speed_stepsize').value
        self.declare_parameter('lap_time_mat_opts.file')
        params_dict['lap_time_mat_opts']['file'] = self.get_parameter(
            'lap_time_mat_opts.file').value
        
        params_dict['plot_opts'] = {}
        self.declare_parameter('plot_opts.mincurv_curv_lin')
        params_dict['plot_opts']['mincurv_curv_lin'] = self.get_parameter(
            'plot_opts.mincurv_curv_lin').value
        self.declare_parameter('plot_opts.raceline')
        params_dict['plot_opts']['raceline'] = self.get_parameter(
            'plot_opts.raceline').value
        self.declare_parameter('plot_opts.imported_bounds')
        params_dict['plot_opts']['imported_bounds'] = self.get_parameter(
            'plot_opts.imported_bounds').value
        self.declare_parameter('plot_opts.raceline_curv')
        params_dict['plot_opts']['raceline_curv'] = self.get_parameter(
            'plot_opts.raceline_curv').value
        self.declare_parameter('plot_opts.racetraj_vel')
        params_dict['plot_opts']['racetraj_vel'] = self.get_parameter(
            'plot_opts.racetraj_vel').value
        self.declare_parameter('plot_opts.racetraj_vel_3d')
        params_dict['plot_opts']['racetraj_vel_3d'] = self.get_parameter(
            'plot_opts.racetraj_vel_3d').value
        self.declare_parameter('plot_opts.racetraj_vel_3d_stepsize')
        params_dict['plot_opts']['racetraj_vel_3d_stepsize'] = self.get_parameter(
            'plot_opts.racetraj_vel_3d_stepsize').value
        self.declare_parameter('plot_opts.spline_normals')
        params_dict['plot_opts']['spline_normals'] = self.get_parameter(
            'plot_opts.spline_normals').value
        self.declare_parameter('plot_opts.mintime_plots')
        params_dict['plot_opts']['mintime_plots'] = self.get_parameter(
            'plot_opts.mintime_plots').value
        self.declare_parameter('plot_opts.racetraj_vel_3d_simple')
        params_dict['plot_opts']['racetraj_vel_3d_simple'] = self.get_parameter(
            'plot_opts.racetraj_vel_3d_simple').value
        return params_dict

    def __centerline_sub_callback(self, msg: Marker):
        start_time = time.time()  # Record the start time

        if self.got_it:
            self.get_logger().info('Already got it')
            return

        self.got_it = True
        self.track.set_reftrack(centerline=[
            [
                point.x,
                point.y,
                point.z if point.z != 0 else self.params_dict['misc']['fakeDistance'],
                point.z if point.z != 0 else self.params_dict['misc']['fakeDistance']
            ]
            for point in msg.points
        ])
        self.get_logger().info(f'Saved waypoints ({len(msg.points)})')

        self.elaborateTrackline()
        self.destroy_subscription(self.centerline_sub)

        elapsed_time = time.time() - start_time  # Calculate elapsed time
        self.get_logger().info(
            f'Callback execution time: {elapsed_time:.4f} seconds')

    def elaborateTrackline(self):
        self.get_logger().info('Elaborating Trackline')
        outcome_msg = self.trajectory.optimize(self.track.get_reftrack())

        if self.params_dict['misc']['savePointsPath']:
            self.track.points_to_file(
                self.params_dict['misc']['savePointsPath'])

        self.get_logger().info(f'{outcome_msg}')

        output = self.trajectory.get_trajectory_opt()
        if (self.params_dict["misc"]["debug"]):
            self.get_logger().info(f'{output}')

        # Resampling the trajectory
        out = elaborate_output(output["raceline"], output["heading"], output["speed"])

        points_list = TrajectoryPoints()
        points_list.header.frame_id = 'track'
        points_list.header.stamp = self.get_clock().now().to_msg()
        output_iterator = zip(
                out['x'], out['y'], out['phi'], out['r'], out['s'])
        for x, y, phi, r, s in output_iterator:
            p = TrajectoryPoint(
                pose = Point(x=x, y=y),
                track_yaw = phi,
                radius = r,
                s = s,
                target_speed = 0.0
            )
            points_list.points.append(p)

        self.speed_profile_pub.publish(points_list)
        self.get_logger().info('Published.')


def main(args=None):
    rclpy.init(args=args)

    global_sub = GlobalPlanner()

    rclpy.spin_once(global_sub)

    global_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
