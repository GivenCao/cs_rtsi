#pragma once
#include <iostream>

const std::string NEW_LINE = "\n";
const std::string QUOTATION = "\"";
std::string CS_SCRIPT = "# HEADER_BEGIN" + NEW_LINE +
"    import threading" + NEW_LINE +
"    global lock" + NEW_LINE +
"    lock = threading.Lock()" + NEW_LINE +
"    global reg_offset_float" + NEW_LINE +
"    global reg_offset_int" + NEW_LINE +
"    global force_mode_type" + NEW_LINE +
"    global selection_vector" + NEW_LINE +
"    global task_frame" + NEW_LINE +
"    global wrench" + NEW_LINE +
"    global limits" + NEW_LINE +
"    global is_servoing" + NEW_LINE +
"    global is_speeding" + NEW_LINE +
"    global is_in_forcemode" + NEW_LINE +
"" + NEW_LINE +
"    reg_offset_float = 0" + NEW_LINE +
"    reg_offset_int = 0" + NEW_LINE +
"    force_mode_type = 2" + NEW_LINE +
"    selection_vector = [0, 0, 0, 0, 0, 0]" + NEW_LINE +
"    task_frame = [0, 0, 0, 0, 0, 0]" + NEW_LINE +
"    wrench = [0, 0, 0, 0, 0, 0]" + NEW_LINE +
"    limits = [0, 0, 0, 0, 0, 0]" + NEW_LINE +
"    is_servoing = 0" + NEW_LINE +
"    is_speeding = 0" + NEW_LINE +
"    is_in_forcemode = 0" + NEW_LINE +
"" + NEW_LINE +
"    global servo_target" + NEW_LINE +
"    global servo_time" + NEW_LINE +
"    global servo_lookahead_time" + NEW_LINE +
"    global servo_gain" + NEW_LINE +
"" + NEW_LINE +
"    servo_target = [0, 0, 0, 0, 0, 0]" + NEW_LINE +
"    servo_time = 0.002" + NEW_LINE +
"    servo_lookahead_time = 0.1" + NEW_LINE +
"    servo_gain = 300" + NEW_LINE +
"" + NEW_LINE +
"    global speed_type" + NEW_LINE +
"    global speed_target" + NEW_LINE +
"    global speed_acceleration" + NEW_LINE +
"    global speed_time" + NEW_LINE +
"" + NEW_LINE +
"    speed_type = 0" + NEW_LINE +
"    speed_target = [0, 0, 0, 0, 0, 0]" + NEW_LINE +
"    speed_acceleration = 0.5" + NEW_LINE +
"    speed_time = 0.5" + NEW_LINE +
"" + NEW_LINE +
"    global move_thrd" + NEW_LINE +
"    global move_type" + NEW_LINE +
"    global move_p" + NEW_LINE +
"    global move_q" + NEW_LINE +
"    global move_vel" + NEW_LINE +
"    global move_acc" + NEW_LINE +
"" + NEW_LINE +
"    move_thrd = 0" + NEW_LINE +
"    move_type = 0" + NEW_LINE +
"    move_p = [0, 0, 0, 0, 0, 0]" + NEW_LINE +
"    move_q = [0, 0, 0, 0, 0, 0]" + NEW_LINE +
"    move_vel = 1.2" + NEW_LINE +
"    move_acc = 0.25" + NEW_LINE +
"" + NEW_LINE +
"    global servo_thrd" + NEW_LINE +
"    global speed_thrd" + NEW_LINE +
"    global force_thrd" + NEW_LINE +
"" + NEW_LINE +
"    servo_thrd = 0" + NEW_LINE +
"    speed_thrd = 0" + NEW_LINE +
"    force_thrd = 0" + NEW_LINE +
"" + NEW_LINE +
"    global jog_feature# 0 - base, 1 - tool, 2 - custom" + NEW_LINE +
"    global jog_speed_vector_dt" + NEW_LINE +
"    global jog_start_pose" + NEW_LINE +
"    global jog_thrd" + NEW_LINE +
"    global jog_dt" + NEW_LINE +
"    global jog_state # 0 - inactive, 1 - new setpoint, 2 - jogging, 3 - stopping (decelerating)" + NEW_LINE +
"" + NEW_LINE +
"    jog_feature = 0 # 0 - base, 1 - tool, 2 - custom" + NEW_LINE +
"    jog_speed_vector_dt = [0, 0, 0, 0, 0, 0]" + NEW_LINE +
"    jog_start_pose = [0, 0, 0, 0, 0, 0]" + NEW_LINE +
"    jog_thrd = 0" + NEW_LINE +
"    jog_dt = 0.002" + NEW_LINE +
"    jog_state = 0 # 0 - inactive, 1 - new setpoint, 2 - jogging, 3 - stopping (decelerating)" + NEW_LINE +
"" + NEW_LINE +
"    def read_input_float_reg(register_index):" + NEW_LINE +
"        return read_input_float_register(register_index + reg_offset_float)" + NEW_LINE +
"    end" + NEW_LINE +
"" + NEW_LINE +
"    def read_input_integer_reg(register_index):" + NEW_LINE +
"        return read_input_integer_register(register_index + reg_offset_int)" + NEW_LINE +
"    end" + NEW_LINE +
"" + NEW_LINE +
"    def write_output_float_reg(register_index, value):" + NEW_LINE +
"        write_output_float_register(register_index + reg_offset_float, value)" + NEW_LINE +
"    end" + NEW_LINE +
"" + NEW_LINE +
"    def write_output_integer_reg(register_index, value):" + NEW_LINE +
"        write_output_integer_register(register_index + reg_offset_int, value)" + NEW_LINE +
"    end" + NEW_LINE +
"" + NEW_LINE +
"    global async_finished" + NEW_LINE +
"    async_finished = -2" + NEW_LINE +
"" + NEW_LINE +
"    def exec_move_path():" + NEW_LINE +
"        textmsg(" + QUOTATION + "exec_move_path" + QUOTATION + ")" + NEW_LINE +
"      # inject move path" + NEW_LINE +
"    end" + NEW_LINE +
"" + NEW_LINE +
"    def signal_async_progress(value):" + NEW_LINE +
"        write_output_integer_register(2, value)" + NEW_LINE +
"    end" + NEW_LINE +
"" + NEW_LINE +
"    def signal_async_operation_started():" + NEW_LINE +
"        write_output_integer_register(2, 0) # 0 indicates operation started" + NEW_LINE +
"    end" + NEW_LINE +
"" + NEW_LINE +
"    def signal_async_operation_finished():" + NEW_LINE +
"        global async_finished" + NEW_LINE +
"        if async_finished == -1:" + NEW_LINE +
"            async_finished = -2" + NEW_LINE +
"        else:" + NEW_LINE +
"            async_finished = -1" + NEW_LINE +
"        end" + NEW_LINE +
"        write_output_integer_register(2, async_finished) # negative values indicate operation finished" + NEW_LINE +
"    end" + NEW_LINE +
"" + NEW_LINE +
"    def reset_async_progress():" + NEW_LINE +
"        global async_finished" + NEW_LINE +
"        async_finished = -2" + NEW_LINE +
"        signal_async_operation_finished()" + NEW_LINE +
"    end" + NEW_LINE +
"" + NEW_LINE +
"    def move_thread():" + NEW_LINE +
"        global move_thrd" + NEW_LINE +
"        global move_type" + NEW_LINE +
"        textmsg(" + QUOTATION + "move_thread started" + QUOTATION + ")" + NEW_LINE +
"        signal_async_operation_started()" + NEW_LINE +
"        while (True):" + NEW_LINE +
"            if move_type == 0 or move_type == 1:" + NEW_LINE +
"                movej(move_q, a=move_acc, v=move_vel)" + NEW_LINE +
"            elif move_type == 2:" + NEW_LINE +
"                movel(move_p, a=move_acc, v=move_vel)" + NEW_LINE +
"            elif move_type == 3:" + NEW_LINE +
"                movel(move_q, a=move_acc, v=move_vel)" + NEW_LINE +
"            elif move_type == 4:" + NEW_LINE +
"                exec_move_path()" + NEW_LINE +
"            end" + NEW_LINE +
"            lock.acquire()" + NEW_LINE +
"            move_thrd = 0" + NEW_LINE +
"            textmsg(" + QUOTATION + "move_thread finished" + QUOTATION + ")" + NEW_LINE +
"            lock.release()" + NEW_LINE +
"            break" + NEW_LINE +
"        end" + NEW_LINE +
"        signal_async_operation_finished()" + NEW_LINE +
"    end" + NEW_LINE +
"" + NEW_LINE +
"    def stop_async_move():" + NEW_LINE +
"        global move_thrd" + NEW_LINE +
"        lock.acquire()" + NEW_LINE +
"        if move_thrd != 0:" + NEW_LINE +
"            textmsg(" + QUOTATION + "stopping async movement - killing move_thrd" + QUOTATION + ")" + NEW_LINE +
"            stop_thread(move_thrd)" + NEW_LINE +
"            move_thrd = 0" + NEW_LINE +
"        end" + NEW_LINE +
"        lock.release()" + NEW_LINE +
"    end" + NEW_LINE +
"" + NEW_LINE +
"    def force_thread():" + NEW_LINE +
"        while (True):" + NEW_LINE +
"            force_mode(task_frame, selection_vector, wrench, force_mode_type, limits)" + NEW_LINE +
"            sync()" + NEW_LINE +
"        end" + NEW_LINE +
"    end" + NEW_LINE +
"" + NEW_LINE +
"    def speed_thread():" + NEW_LINE +
"        while (True):" + NEW_LINE +
"            lock.acquire()" + NEW_LINE +
"            type = speed_type" + NEW_LINE +
"            target = speed_target" + NEW_LINE +
"            acceleration = speed_acceleration" + NEW_LINE +
"            time = speed_time" + NEW_LINE +
"            lock.release()" + NEW_LINE +
"            if type == 0:" + NEW_LINE +
"                if time > 0:" + NEW_LINE +
"                    speedl(target, a=acceleration, t=time)" + NEW_LINE +
"                else:" + NEW_LINE +
"                    speedl(target, a=acceleration)" + NEW_LINE +
"                end" + NEW_LINE +
"            else:" + NEW_LINE +
"                if time > 0:" + NEW_LINE +
"                    speedj(target, a=acceleration, t=time)" + NEW_LINE +
"                else:" + NEW_LINE +
"                    speedj(target, a=acceleration)" + NEW_LINE +
"                end" + NEW_LINE +
"            end" + NEW_LINE +
"        end" + NEW_LINE +
"    end" + NEW_LINE +
"" + NEW_LINE +
"    def servo_thread():" + NEW_LINE +
"        while (True):" + NEW_LINE +
"            lock.acquire()" + NEW_LINE +
"            q = servo_target" + NEW_LINE +
"            dt = servo_time" + NEW_LINE +
"            lh_time = servo_lookahead_time" + NEW_LINE +
"            g = servo_gain" + NEW_LINE +
"            lock.release()" + NEW_LINE +
"            servoj(q, t=dt, lookahead_time=lh_time, gain=g)" + NEW_LINE +
"        end" + NEW_LINE +
"    end" + NEW_LINE +
"" + NEW_LINE +
"    def q_from_input_float_registers(register_index):" + NEW_LINE +
"        q = [0, 0, 0, 0, 0, 0]" + NEW_LINE +
"        q[0] = read_input_float_reg(register_index + 0)" + NEW_LINE +
"        q[1] = read_input_float_reg(register_index + 1)" + NEW_LINE +
"        q[2] = read_input_float_reg(register_index + 2)" + NEW_LINE +
"        q[3] = read_input_float_reg(register_index + 3)" + NEW_LINE +
"        q[4] = read_input_float_reg(register_index + 4)" + NEW_LINE +
"        q[5] = read_input_float_reg(register_index + 5)" + NEW_LINE +
"        return q" + NEW_LINE +
"    end" + NEW_LINE +
"" + NEW_LINE +
"    def pose_from_input_float_registers(register_index):" + NEW_LINE +
"        pose = [0, 0, 0, 0, 0, 0]" + NEW_LINE +
"        pose[0] = read_input_float_reg(register_index + 0)" + NEW_LINE +
"        pose[1] = read_input_float_reg(register_index + 1)" + NEW_LINE +
"        pose[2] = read_input_float_reg(register_index + 2)" + NEW_LINE +
"        pose[3] = read_input_float_reg(register_index + 3)" + NEW_LINE +
"        pose[4] = read_input_float_reg(register_index + 4)" + NEW_LINE +
"        pose[5] = read_input_float_reg(register_index + 5)" + NEW_LINE +
"        return pose" + NEW_LINE +
"    end" + NEW_LINE +
"" + NEW_LINE +
"    def pose_to_output_float_registers(register_index, pose):" + NEW_LINE +
"        write_output_float_reg(register_index + 0, pose[0])" + NEW_LINE +
"        write_output_float_reg(register_index + 1, pose[1])" + NEW_LINE +
"        write_output_float_reg(register_index + 2, pose[2])" + NEW_LINE +
"        write_output_float_reg(register_index + 3, pose[3])" + NEW_LINE +
"        write_output_float_reg(register_index + 4, pose[4])" + NEW_LINE +
"        write_output_float_reg(register_index + 5, pose[5])" + NEW_LINE +
"    end" + NEW_LINE +
"" + NEW_LINE +
"    def q_to_output_float_registers(register_index, q):" + NEW_LINE +
"        write_output_float_reg(register_index + 0, q[0])" + NEW_LINE +
"        write_output_float_reg(register_index + 1, q[1])" + NEW_LINE +
"        write_output_float_reg(register_index + 2, q[2])" + NEW_LINE +
"        write_output_float_reg(register_index + 3, q[3])" + NEW_LINE +
"        write_output_float_reg(register_index + 4, q[4])" + NEW_LINE +
"        write_output_float_reg(register_index + 5, q[5])" + NEW_LINE +
"    end" + NEW_LINE +
"" + NEW_LINE +
"    def mas_cog_output_float_registers(register_index, mas_cog):" + NEW_LINE +
"        write_output_float_reg(register_index + 0, mas_cog[0])" + NEW_LINE +
"        write_output_float_reg(register_index + 1, mas_cog[1])" + NEW_LINE +
"        write_output_float_reg(register_index + 2, mas_cog[2])" + NEW_LINE +
"        write_output_float_reg(register_index + 3, mas_cog[3])" + NEW_LINE +
"    end" + NEW_LINE +
"" + NEW_LINE +
"    # Returs a pose that contains only the translation part of a given pose" + NEW_LINE +
"    def get_pose_translation(pose):" + NEW_LINE +
"        return [pose[0], pose[1], pose[2], 0, 0, 0]" + NEW_LINE +
"    end" + NEW_LINE +
"  " + NEW_LINE +
"    # Returns a pose that contains only the rotation part of a given pose" + NEW_LINE +
"    def get_pose_rotation(pose):" + NEW_LINE +
"        return [0, 0, 0, pose[3], pose[4], pose[5]]" + NEW_LINE +
"    end" + NEW_LINE +
"" + NEW_LINE +
"    # scales the given pose by the given factor" + NEW_LINE +
"    def scale_pose(p, factor):" + NEW_LINE +
"        p[0] = p[0] * factor" + NEW_LINE +
"        p[1] = p[1] * factor" + NEW_LINE +
"        p[2] = p[2] * factor" + NEW_LINE +
"        p[3] = p[3] * factor" + NEW_LINE +
"        p[4] = p[4] * factor" + NEW_LINE +
"        p[5] = p[5] * factor" + NEW_LINE +
"        return p" + NEW_LINE +
"    end" + NEW_LINE +
"" + NEW_LINE +
"    # The jog thread provides jogging functionality similar to the one you" + NEW_LINE +
"    # find in the teach pendant." + NEW_LINE +
"    def jog_thread():" + NEW_LINE +
"        textmsg(" + QUOTATION + "jog_thread started" + QUOTATION + ")" + NEW_LINE +
"        # We latch the global jog settings. The feature setting is only latched" + NEW_LINE +
"        # here. That means, it is not possible to change the feature as soon as the" + NEW_LINE +
"        # move started. To change the feature, you first have to stop the move" + NEW_LINE +
"        lock.acquire()" + NEW_LINE +
"        start_pose = jog_start_pose" + NEW_LINE +
"        feature = jog_feature" + NEW_LINE +
"        lock.release()" + NEW_LINE +
"" + NEW_LINE +
"        # Initialize target_pose depending on feature" + NEW_LINE +
"        if feature == 0: # base feature" + NEW_LINE +
"            target_pose = start_pose" + NEW_LINE +
"        elif feature == 1: # tool feature" + NEW_LINE +
"            target_pose = [0, 0, 0, 0, 0, 0]" + NEW_LINE +
"        end" + NEW_LINE +
"" + NEW_LINE +
"        while (True):" + NEW_LINE +
"            lock.acquire()" + NEW_LINE +
"            if jog_state == 1: # 1 = new setpoint" + NEW_LINE +
"                speeds = jog_speed_vector_dt" + NEW_LINE +
"                jog_state = 2 # 2 = jogging" + NEW_LINE +
"                textmsg(" + QUOTATION + "new jog setpoint" + QUOTATION + ")" + NEW_LINE +
"            end" + NEW_LINE +
"            lock.release()" + NEW_LINE +
"" + NEW_LINE +
"            if feature == 0: # base feature" + NEW_LINE +
"                target_transl = get_pose_translation(pose_add(target_pose, speeds))" + NEW_LINE +
"                speeds_rot = get_pose_rotation(speeds)" + NEW_LINE +
"                target_rot = pose_trans(speeds_rot, get_pose_rotation(target_pose))" + NEW_LINE +
"                target_pose = pose_add(target_transl, target_rot)" + NEW_LINE +
"                q = get_inverse_kin(target_pose)" + NEW_LINE +
"            elif feature == 1: # tool feature" + NEW_LINE +
"                target_pose = pose_add(target_pose, speeds)" + NEW_LINE +
"                pose_wrt_base = pose_trans(start_pose, target_pose)" + NEW_LINE +
"                q = get_inverse_kin(pose_wrt_base)" + NEW_LINE +
"            end" + NEW_LINE +
"            servoj(q, jog_dt, lookahead_time=0.03, gain=1000)" + NEW_LINE +
"        end" + NEW_LINE +
"    end" + NEW_LINE +
"" + NEW_LINE +
"    def signal_ready():" + NEW_LINE +
"        write_output_integer_reg(0, 1)" + NEW_LINE +
"    end" + NEW_LINE +
"" + NEW_LINE +
"    def signal_done_with_cmd():" + NEW_LINE +
"        write_output_integer_reg(0, 2)" + NEW_LINE +
"    end" + NEW_LINE +
"" + NEW_LINE +
"    def rtde_cmd():" + NEW_LINE +
"        return read_input_integer_reg(0)" + NEW_LINE +
"    end" + NEW_LINE +
"" + NEW_LINE +
"    def process_cmd():" + NEW_LINE +
"        global reg_offset_float" + NEW_LINE +
"        global reg_offset_int" + NEW_LINE +
"        global force_mode_type" + NEW_LINE +
"        global selection_vector" + NEW_LINE +
"        global task_frame" + NEW_LINE +
"        global wrench" + NEW_LINE +
"        global limits" + NEW_LINE +
"        global is_servoing" + NEW_LINE +
"        global is_speeding" + NEW_LINE +
"        global is_in_forcemode" + NEW_LINE +
"" + NEW_LINE +
"        global servo_target" + NEW_LINE +
"        global servo_time" + NEW_LINE +
"        global servo_lookahead_time" + NEW_LINE +
"        global servo_gain" + NEW_LINE +
"" + NEW_LINE +
"        global speed_type" + NEW_LINE +
"        global speed_target" + NEW_LINE +
"        global speed_acceleration" + NEW_LINE +
"        global speed_time" + NEW_LINE +
"" + NEW_LINE +
"        global move_thrd" + NEW_LINE +
"        global move_type" + NEW_LINE +
"        global move_p" + NEW_LINE +
"        global move_q" + NEW_LINE +
"        global move_vel" + NEW_LINE +
"        global move_acc" + NEW_LINE +
"" + NEW_LINE +
"        global servo_thrd" + NEW_LINE +
"        global speed_thrd" + NEW_LINE +
"        global force_thrd" + NEW_LINE +
"" + NEW_LINE +
"        global jog_feature# 0 - base, 1 - tool, 2 - custom" + NEW_LINE +
"        global jog_speed_vector_dt" + NEW_LINE +
"        global jog_start_pose" + NEW_LINE +
"        global jog_thrd" + NEW_LINE +
"        global jog_dt" + NEW_LINE +
"        global jog_state # 0 - inactive, 1 - new setpoint, 2 - jogging, 3 - stopping (decelerating)" + NEW_LINE +
"" + NEW_LINE +
"        cmd = read_input_integer_reg(0)" + NEW_LINE +
"" + NEW_LINE +
"        if cmd == 1:" + NEW_LINE +
"            textmsg(" + QUOTATION + "movej" + QUOTATION + ")" + NEW_LINE +
"            q = q_from_input_float_registers(0)" + NEW_LINE +
"            velocity = read_input_float_reg(6)" + NEW_LINE +
"            acceleration = read_input_float_reg(7)" + NEW_LINE +
"            async = read_input_integer_reg(1)" + NEW_LINE +
"            textmsg(" + QUOTATION + "Target q:" + QUOTATION + ")" + NEW_LINE +
"            textmsg(q)" + NEW_LINE +
"            stop_async_move()" + NEW_LINE +
"            if async == 1:" + NEW_LINE +
"                lock.acquire()" + NEW_LINE +
"                move_type = 0" + NEW_LINE +
"                move_q = q" + NEW_LINE +
"                move_acc = acceleration" + NEW_LINE +
"                move_vel = velocity" + NEW_LINE +
"                lock.release()" + NEW_LINE +
"                move_thrd = start_thread(move_thread,())" + NEW_LINE +
"            else:" + NEW_LINE +
"                movej(q, a=acceleration, v=velocity)" + NEW_LINE +
"                textmsg(" + QUOTATION + "movej done" + QUOTATION + ")" + NEW_LINE +
"            end" + NEW_LINE +
"        elif cmd == 2:" + NEW_LINE +
"            textmsg(" + QUOTATION + "movej_ik" + QUOTATION + ")" + NEW_LINE +
"            pose = pose_from_input_float_registers(0)" + NEW_LINE +
"            velocity = read_input_float_reg(6)" + NEW_LINE +
"            acceleration = read_input_float_reg(7)" + NEW_LINE +
"            async = read_input_integer_reg(1)" + NEW_LINE +
"            textmsg(" + QUOTATION + "Target pose:" + QUOTATION + ")" + NEW_LINE +
"            textmsg(pose)" + NEW_LINE +
"            q = get_inverse_kin(pose)" + NEW_LINE +
"            textmsg(" + QUOTATION + "Target q:" + QUOTATION + ")" + NEW_LINE +
"            textmsg(q)" + NEW_LINE +
"            stop_async_move()" + NEW_LINE +
"            if async == 1:" + NEW_LINE +
"                lock.acquire()" + NEW_LINE +
"                move_type = 1" + NEW_LINE +
"                move_q = q" + NEW_LINE +
"                move_acc = acceleration" + NEW_LINE +
"                move_vel = velocity" + NEW_LINE +
"                lock.release()" + NEW_LINE +
"                move_thrd = start_thread(move_thread,())" + NEW_LINE +
"            else:" + NEW_LINE +
"                movej(q, a=acceleration, v=velocity)" + NEW_LINE +
"                textmsg(" + QUOTATION + "movej_ik done" + QUOTATION + ")" + NEW_LINE +
"            end" + NEW_LINE +
"        elif cmd == 3:" + NEW_LINE +
"            textmsg(" + QUOTATION + "movel" + QUOTATION + ")" + NEW_LINE +
"            pose = pose_from_input_float_registers(0)" + NEW_LINE +
"            velocity = read_input_float_reg(6)" + NEW_LINE +
"            acceleration = read_input_float_reg(7)" + NEW_LINE +
"            async = read_input_integer_reg(1)" + NEW_LINE +
"            textmsg(" + QUOTATION + "Target pose:" + QUOTATION + ")" + NEW_LINE +
"            textmsg(pose)" + NEW_LINE +
"            stop_async_move()" + NEW_LINE +
"            if async == 1:" + NEW_LINE +
"                lock.acquire()" + NEW_LINE +
"                move_type = 2" + NEW_LINE +
"                move_p = pose" + NEW_LINE +
"                move_acc = acceleration" + NEW_LINE +
"                move_vel = velocity" + NEW_LINE +
"                lock.release()" + NEW_LINE +
"                move_thrd = start_thread(move_thread,())" + NEW_LINE +
"            else:" + NEW_LINE +
"                movel(pose, a=acceleration, v=velocity)" + NEW_LINE +
"                textmsg(" + QUOTATION + "movel done" + QUOTATION + ")" + NEW_LINE +
"            end" + NEW_LINE +
"        elif cmd == 4:" + NEW_LINE +
"            textmsg(" + QUOTATION + "movel_fk" + QUOTATION + ")" + NEW_LINE +
"            q = q_from_input_float_registers(0)" + NEW_LINE +
"            velocity = read_input_float_reg(6)" + NEW_LINE +
"            acceleration = read_input_float_reg(7)" + NEW_LINE +
"            async = read_input_integer_reg(1)" + NEW_LINE +
"            textmsg(" + QUOTATION + "Target q:" + QUOTATION + ")" + NEW_LINE +
"            textmsg(q)" + NEW_LINE +
"            stop_async_move()" + NEW_LINE +
"            if async == 1:" + NEW_LINE +
"                lock.acquire()" + NEW_LINE +
"                move_type = 3" + NEW_LINE +
"                move_q = q" + NEW_LINE +
"                move_acc = acceleration" + NEW_LINE +
"                move_vel = velocity" + NEW_LINE +
"                lock.release()" + NEW_LINE +
"                move_thrd = start_thread(move_thread,())" + NEW_LINE +
"            else:" + NEW_LINE +
"                movel(q, a=acceleration, v=velocity)" + NEW_LINE +
"                textmsg(" + QUOTATION + "movel_fk done" + QUOTATION + ")" + NEW_LINE +
"            end" + NEW_LINE +
"        elif cmd == 6:" + NEW_LINE +
"            # force_mode" + NEW_LINE +
"            lock.acquire()" + NEW_LINE +
"            force_mode_type = read_input_integer_reg(1)" + NEW_LINE +
"            selection_vector[0] = read_input_integer_reg(2)" + NEW_LINE +
"            selection_vector[1] = read_input_integer_reg(3)" + NEW_LINE +
"            selection_vector[2] = read_input_integer_reg(4)" + NEW_LINE +
"            selection_vector[3] = read_input_integer_reg(5)" + NEW_LINE +
"            selection_vector[4] = read_input_integer_reg(6)" + NEW_LINE +
"            selection_vector[5] = read_input_integer_reg(7)" + NEW_LINE +
"" + NEW_LINE +
"            task_frame = pose_from_input_float_registers(0)" + NEW_LINE +
"            wrench = q_from_input_float_registers(6)" + NEW_LINE +
"            limits = q_from_input_float_registers(12)" + NEW_LINE +
"            lock.release()" + NEW_LINE +
"" + NEW_LINE +
"            if is_in_forcemode == 0:" + NEW_LINE +
"                is_in_forcemode = 1" + NEW_LINE +
"                if force_thrd == 0:" + NEW_LINE +
"                    force_thrd = start_thread(force_thread,())" + NEW_LINE +
"                end" + NEW_LINE +
"            end" + NEW_LINE +
"        elif cmd == 7:" + NEW_LINE +
"            textmsg(" + QUOTATION + "force_mode_stop" + QUOTATION + ")" + NEW_LINE +
"            lock.acquire()" + NEW_LINE +
"            is_in_forcemode = 0" + NEW_LINE +
"            stop_thread(force_thrd)" + NEW_LINE +
"            force_thrd = 0" + NEW_LINE +
"            end_force_mode()" + NEW_LINE +
"            stopl(10)" + NEW_LINE +
"            lock.release()" + NEW_LINE +
"            textmsg(" + QUOTATION + "force_mode stopped" + QUOTATION + ")" + NEW_LINE +
"        elif cmd == 8:" + NEW_LINE +
"            textmsg(" + QUOTATION + "zero_ftsensor" + QUOTATION + ")" + NEW_LINE +
"            zero_ftsensor()" + NEW_LINE +
"            textmsg(" + QUOTATION + "ftsensor zeroed" + QUOTATION + ")" + NEW_LINE +
"        elif cmd == 9:" + NEW_LINE +
"            # speedJ" + NEW_LINE +
"            qd = q_from_input_float_registers(0)" + NEW_LINE +
"" + NEW_LINE +
"            lock.acquire()" + NEW_LINE +
"            speed_type = 1" + NEW_LINE +
"            speed_acceleration = read_input_float_reg(6)" + NEW_LINE +
"            speed_time = read_input_float_reg(7)" + NEW_LINE +
"            speed_target = qd" + NEW_LINE +
"            lock.release()" + NEW_LINE +
"" + NEW_LINE +
"            if is_speeding == 0:" + NEW_LINE +
"                lock.acquire()" + NEW_LINE +
"                is_speeding = 1" + NEW_LINE +
"                lock.release()" + NEW_LINE +
"                if speed_thrd == 0:" + NEW_LINE +
"                    speed_thrd = start_thread(speed_thread,())" + NEW_LINE +
"                end" + NEW_LINE +
"            end" + NEW_LINE +
"        elif cmd == 10:" + NEW_LINE +
"            # speedL" + NEW_LINE +
"            xd = q_from_input_float_registers(0)" + NEW_LINE +
"            lock.acquire()" + NEW_LINE +
"            speed_type = 0" + NEW_LINE +
"            speed_acceleration = read_input_float_reg(6)" + NEW_LINE +
"            speed_time = read_input_float_reg(7)" + NEW_LINE +
"            speed_target = xd" + NEW_LINE +
"            lock.release()" + NEW_LINE +
"" + NEW_LINE +
"            if is_speeding == 0:" + NEW_LINE +
"                is_speeding = 1" + NEW_LINE +
"                if speed_thrd == 0:" + NEW_LINE +
"                    speed_thrd = start_thread(speed_thread,())" + NEW_LINE +
"                end" + NEW_LINE +
"            end" + NEW_LINE +
"        elif cmd == 11:" + NEW_LINE +
"            # servoJ" + NEW_LINE +
"            q = q_from_input_float_registers(0)" + NEW_LINE +
"            velocity = read_input_float_reg(6)" + NEW_LINE +
"            acceleration = read_input_float_reg(7)" + NEW_LINE +
"" + NEW_LINE +
"            lock.acquire()" + NEW_LINE +
"            servo_target = q" + NEW_LINE +
"            textmsg(" + QUOTATION + "servo target:" + QUOTATION + ",q)" + NEW_LINE +
"            servo_time = read_input_float_reg(8)" + NEW_LINE +
"            servo_lookahead_time = read_input_float_reg(9)" + NEW_LINE +
"            servo_gain = read_input_float_reg(10)" + NEW_LINE +
"            lock.release()" + NEW_LINE +
"" + NEW_LINE +
"            if is_servoing == 0:" + NEW_LINE +
"                is_servoing = 1" + NEW_LINE +
"                if servo_thrd == 0:" + NEW_LINE +
"                    servo_thrd = start_thread(servo_thread,())" + NEW_LINE +
"                end" + NEW_LINE +
"            end" + NEW_LINE +
"            " + NEW_LINE +
//"        elif cmd == 12:" + NEW_LINE +
//"            # servoC" + NEW_LINE +
//"            pose = pose_from_input_float_registers(0)" + NEW_LINE +
//"            velocity = read_input_float_reg(6)" + NEW_LINE +
//"            acceleration = read_input_float_reg(7)" + NEW_LINE +
//"            blend = read_input_float_reg(8)" + NEW_LINE +
//"" + NEW_LINE +
//"            lock.acquire()" + NEW_LINE +
//"            servoc_target = pose" + NEW_LINE +
//"            servoc_acceleration = acceleration" + NEW_LINE +
//"            servoc_velocity = velocity" + NEW_LINE +
//"            servoc_blend = blend" + NEW_LINE +
//"            lock.release()" + NEW_LINE +
//"" + NEW_LINE +
//"            if is_servoing == 0:" + NEW_LINE +
//"                is_servoing = 1" + NEW_LINE +
//"                if servoc_thrd == 0:" + NEW_LINE +
//"                    global servoc_thrd = run servoc_thread()" + NEW_LINE +
//"                end" + NEW_LINE +
//"            end" + NEW_LINE +
//"" + NEW_LINE +
"        elif cmd == 15:" + NEW_LINE +
"            textmsg(" + QUOTATION + "speed_stop" + QUOTATION + ")" + NEW_LINE +
"            deceleration_rate = read_input_float_reg(0)" + NEW_LINE +
"            lock.acquire()" + NEW_LINE +
"            is_speeding = 0" + NEW_LINE +
"            stop_thread(speed_thrd)" + NEW_LINE +
"            speed_thrd = 0" + NEW_LINE +
"            if speed_type == 0:" + NEW_LINE +
"                stopl(deceleration_rate)" + NEW_LINE +
"            else:" + NEW_LINE +
"                stopj(deceleration_rate)" + NEW_LINE +
"            end" + NEW_LINE +
"            lock.release()" + NEW_LINE +
"            textmsg(" + QUOTATION + "speed_stop done" + QUOTATION + ")" + NEW_LINE +
"        elif cmd == 16:" + NEW_LINE +
"            textmsg(" + QUOTATION + "servo_stop" + QUOTATION + ")" + NEW_LINE +
"            deceleration_rate = read_input_float_reg(0)" + NEW_LINE +
"            lock.acquire()" + NEW_LINE +
"            is_servoing = 0" + NEW_LINE +
"            stop_thread(servo_thrd)" + NEW_LINE +
"            servo_thrd = 0" + NEW_LINE +
"            lock.release()" + NEW_LINE +
"            stopl(deceleration_rate)" + NEW_LINE +
"            textmsg(" + QUOTATION + "servo_stop done" + QUOTATION + ")" + NEW_LINE +
"        elif cmd == 17:" + NEW_LINE +
"            textmsg(" + QUOTATION + "set_payload" + QUOTATION + ")" + NEW_LINE +
"            mass = read_input_float_reg(0)" + NEW_LINE +
"            cog_x = read_input_float_reg(1)" + NEW_LINE +
"            cog_y = read_input_float_reg(2)" + NEW_LINE +
"            cog_z = read_input_float_reg(3)" + NEW_LINE +
"            cog = [cog_x, cog_y, cog_z]" + NEW_LINE +
"            if cog_x == 0 and cog_y == 0 and cog_z == 0:" + NEW_LINE +
"                set_payload(mass, get_target_payload_cog())" + NEW_LINE +
"            else:" + NEW_LINE +
"                set_payload(mass, cog)" + NEW_LINE +
"            end" + NEW_LINE +
"            textmsg(" + QUOTATION + "active payload:" + QUOTATION + ")" + NEW_LINE +
"            textmsg(get_target_payload_mass())" + NEW_LINE +
"            textmsg(get_target_payload_cog())" + NEW_LINE +
"            textmsg(" + QUOTATION + "set_payload done" + QUOTATION + ")" + NEW_LINE +
//"        elif cmd == 18:" + NEW_LINE +
//"            textmsg(" + QUOTATION + "teach_mode" + QUOTATION + ")" + NEW_LINE +
//"            teach_mode()" + NEW_LINE +
//"            textmsg(" + QUOTATION + "teach_mode done" + QUOTATION + ")" + NEW_LINE +
//"        elif cmd == 19:" + NEW_LINE +
//"            textmsg(" + QUOTATION + "end_teach_mode" + QUOTATION + ")" + NEW_LINE +
//"            end_teach_mode()" + NEW_LINE +
//"            textmsg(" + QUOTATION + "end_teach_mode done" + QUOTATION + ")" + NEW_LINE +
//"        elif cmd == 20:" + NEW_LINE +
//"            textmsg(" + QUOTATION + "force_mode_set_damping" + QUOTATION + ")" + NEW_LINE +
//"            damping = read_input_float_reg(0)" + NEW_LINE +
//"$V35        force_mode_set_damping(damping)" + NEW_LINE +
//"            textmsg(" + QUOTATION + "force_mode_set_damping done" + QUOTATION + ")" + NEW_LINE +
//"        elif cmd == 21:" + NEW_LINE +
//"            textmsg(" + QUOTATION + "force_mode_set_gain_scaling" + QUOTATION + ")" + NEW_LINE +
//"            scaling = read_input_float_reg(0)" + NEW_LINE +
//"$V50        force_mode_set_gain_scaling(scaling)" + NEW_LINE +
//"            textmsg(" + QUOTATION + "force_mode_set_gain_scaling done" + QUOTATION + ")" + NEW_LINE +
"        elif cmd == 24:" + NEW_LINE +
"            pose = pose_from_input_float_registers(0)" + NEW_LINE +
"            velocity = read_input_float_reg(6)" + NEW_LINE +
"            acceleration = read_input_float_reg(7)" + NEW_LINE +
"            q = get_inverse_kin(pose)" + NEW_LINE +
"" + NEW_LINE +
"            lock.acquire()" + NEW_LINE +
"            servo_target = q" + NEW_LINE +
"            servo_time = read_input_float_reg(8)" + NEW_LINE +
"            servo_lookahead_time = read_input_float_reg(9)" + NEW_LINE +
"            servo_gain = read_input_float_reg(10)" + NEW_LINE +
"            lock.release()" + NEW_LINE +
"" + NEW_LINE +
"            if is_servoing == 0:" + NEW_LINE +
"                is_servoing = 1" + NEW_LINE +
"                if servo_thrd == 0:" + NEW_LINE +
"                    servo_thrd = start_thread(servo_thread,())" + NEW_LINE +
"                end" + NEW_LINE +
"            end" + NEW_LINE +
//"        elif cmd == 25:" + NEW_LINE +
//"            # tool_contact" + NEW_LINE +
//"            direction = pose_from_input_float_registers(0)" + NEW_LINE +
//"$V54        time_steps = tool_contact(direction)" + NEW_LINE +
//"$V54        write_output_integer_reg(1, time_steps)" + NEW_LINE +
"        elif cmd == 26:" + NEW_LINE +
"            # get_steptime" + NEW_LINE +
"            step_time = get_steptime()" + NEW_LINE +
"            write_output_float_reg(0, step_time)" + NEW_LINE +
//"        elif cmd == 27:" + NEW_LINE +
//"            # get_actual_joint_positions_history" + NEW_LINE +
//"            steps = read_input_integer_reg(1)" + NEW_LINE +
//"$V54        joint_positions_history = get_actual_joint_positions_history(steps)" + NEW_LINE +
//"$V54        q_to_output_float_registers(0, joint_positions_history)" + NEW_LINE +
"        elif cmd == 28:" + NEW_LINE +
"            textmsg(" + QUOTATION + "get_target_waypoint" + QUOTATION + ")" + NEW_LINE +
"            target_waypoint = get_target_waypoint()" + NEW_LINE +
"            pose_to_output_float_registers(0, target_waypoint)" + NEW_LINE +
"            textmsg(" + QUOTATION + "get_target_waypoint done" + QUOTATION + ")" + NEW_LINE +
"        elif cmd == 29:" + NEW_LINE +
"            textmsg(" + QUOTATION + "set_tcp" + QUOTATION + ")" + NEW_LINE +
"            pose = pose_from_input_float_registers(0)" + NEW_LINE +
"            set_tcp(pose)" + NEW_LINE +
"            textmsg(" + QUOTATION + "set_tcp done" + QUOTATION + ")" + NEW_LINE +
"        elif cmd == 30:" + NEW_LINE +
"            textmsg(" + QUOTATION + "get_inverse_kin_args" + QUOTATION + ")" + NEW_LINE +
"            x = pose_from_input_float_registers(0)" + NEW_LINE +
"            qnear = q_from_input_float_registers(6)" + NEW_LINE +
"            maxPositionError = read_input_float_reg(12)" + NEW_LINE +
"            maxOrientationError = read_input_float_reg(13)" + NEW_LINE +
"            textmsg(" + QUOTATION + "target pose:" + QUOTATION + ",x)" + NEW_LINE +
"            textmsg(" + QUOTATION + "qnear:" + QUOTATION + ",qnear)" + NEW_LINE +
"" + NEW_LINE +
"            q = get_inverse_kin(x, qnear)" + NEW_LINE +
"            q_to_output_float_registers(0, q)" + NEW_LINE +
"            textmsg(" + QUOTATION + "inverse q:" + QUOTATION + ",q)" + NEW_LINE +
"            textmsg(" + QUOTATION + "get_inverse_kin done" + QUOTATION + ")" + NEW_LINE +
//"        elif cmd == 31:" + NEW_LINE +
//"            textmsg(" + QUOTATION + "protective_stop" + QUOTATION + ")" + NEW_LINE +
//"$V38        protective_stop()" + NEW_LINE +
//"            textmsg(" + QUOTATION + "protective_stop done" + QUOTATION + ")" + NEW_LINE +
"        elif cmd == 33:" + NEW_LINE +
"            textmsg(" + QUOTATION + "stopl" + QUOTATION + ")" + NEW_LINE +
"            deceleration_rate = read_input_float_reg(0)" + NEW_LINE +
"            stop_async_move()" + NEW_LINE +
"            stopl(deceleration_rate)" + NEW_LINE +
"            textmsg(" + QUOTATION + "stopl done" + QUOTATION + ")" + NEW_LINE +
"        elif cmd == 34:" + NEW_LINE +
"            textmsg(" + QUOTATION + "stopj" + QUOTATION + ")" + NEW_LINE +
"            deceleration_rate = read_input_float_reg(0)" + NEW_LINE +
"            stop_async_move()" + NEW_LINE +
"            stopj(deceleration_rate)" + NEW_LINE +
"            textmsg(" + QUOTATION + "stopj done" + QUOTATION + ")" + NEW_LINE +
"        elif cmd == 35:" + NEW_LINE +
"            textmsg(" + QUOTATION + "set_watchdog" + QUOTATION + ")" + NEW_LINE +
"            # Setup watchdog for the RTSI communication" + NEW_LINE +
"            watchdog_min_frequency = read_input_float_reg(0)" + NEW_LINE +
"            if reg_offset_int == 0:" + NEW_LINE +
"                rtsi_set_watchdog(" + QUOTATION + "input_int_register_0" + QUOTATION + ", watchdog_min_frequency, " + QUOTATION + "stop" + QUOTATION + ")" + NEW_LINE +
"            elif reg_offset_int == 24:" + NEW_LINE +
"                rtsi_set_watchdog(" + QUOTATION + "input_int_register_24" + QUOTATION + ", watchdog_min_frequency, " + QUOTATION + "stop" + QUOTATION + ")" + NEW_LINE +
"            else:" + NEW_LINE +
"                rtsi_set_watchdog(" + QUOTATION + "input_int_register_0" + QUOTATION + ", watchdog_min_frequency, " + QUOTATION + "stop" + QUOTATION + ")" + NEW_LINE +
"            end" + NEW_LINE +
"            textmsg(" + QUOTATION + "set_watchdog done" + QUOTATION + ")" + NEW_LINE +
//"        elif cmd == 36:" + NEW_LINE +
//"            textmsg(" + QUOTATION + "is_pose_within_safety_limits" + QUOTATION + ")" + NEW_LINE +
//"            pose = pose_from_input_float_registers(0)" + NEW_LINE +
//"            safe_pose = is_within_safety_limits(pose)" + NEW_LINE +
//"            if safe_pose == True:" + NEW_LINE +
//"               write_output_integer_reg(1, 1)" + NEW_LINE +
//"            else:" + NEW_LINE +
//"               write_output_integer_reg(1, 0)" + NEW_LINE +
////"            end" + NEW_LINE +
//"            textmsg(" + QUOTATION + "is_pose_within_safety_limits done" + QUOTATION + ")" + NEW_LINE +
"        elif cmd == 37:" + NEW_LINE +
"            textmsg(" + QUOTATION + "is_joints_within_safety_limits" + QUOTATION + ")" + NEW_LINE +
"            q = q_from_input_float_registers(0)" + NEW_LINE +
"            safe_q = is_within_safety_limits(q)" + NEW_LINE +
"            if safe_q == True:" + NEW_LINE +
"                write_output_integer_reg(1, 1)" + NEW_LINE +
"            else:" + NEW_LINE +
"                write_output_integer_reg(1, 0)" + NEW_LINE +
"            end" + NEW_LINE +
"            textmsg(" + QUOTATION + "is_joints_within_safety_limits done" + QUOTATION + ")" + NEW_LINE +
"        elif cmd == 38:" + NEW_LINE +
"            # get_joint_torques" + NEW_LINE +
"            torques = get_joint_torques()" + NEW_LINE +
"            q_to_output_float_registers(0, torques)" + NEW_LINE +
"        elif cmd == 39:" + NEW_LINE +
"            textmsg(" + QUOTATION + "pose_trans" + QUOTATION + ")" + NEW_LINE +
"            p_from = pose_from_input_float_registers(0)" + NEW_LINE +
"            p_from_to = pose_from_input_float_registers(6)" + NEW_LINE +
"            p = pose_trans(p_from, p_from_to)" + NEW_LINE +
"            pose_to_output_float_registers(0, p)" + NEW_LINE +
"            textmsg(" + QUOTATION + "pose_trans done" + QUOTATION + ")" + NEW_LINE +
"        elif cmd == 40:" + NEW_LINE +
"            textmsg(" + QUOTATION + "get_tcp_offset" + QUOTATION + ")" + NEW_LINE +
"            tcp_offset = get_tcp_offset()" + NEW_LINE +
"            textmsg(tcp_offset)" + NEW_LINE +
"            pose_to_output_float_registers(0, tcp_offset)" + NEW_LINE +
"            textmsg(" + QUOTATION + "get_tcp_offset done" + QUOTATION + ")" + NEW_LINE +
"        elif cmd == 41:" + NEW_LINE +
"            textmsg(" + QUOTATION + "start_jog" + QUOTATION + ")" + NEW_LINE +
"            lock.acquire()" + NEW_LINE +
"            jog_state = 1 # 3 = stopping (decelerating)" + NEW_LINE +
"            speed_pose = pose_from_input_float_registers(0)" + NEW_LINE +
"            jog_speed_vector_dt = scale_pose(speed_pose, jog_dt)" + NEW_LINE +
"            jog_feature = read_input_float_reg(6)" + NEW_LINE +
"            jog_start_pose = get_actual_tcp_pose()" + NEW_LINE +
"            if jog_thrd == 0:" + NEW_LINE +
"                jog_thrd = start_thread(jog_thread,())" + NEW_LINE +
"            end" + NEW_LINE +
"            lock.release()" + NEW_LINE +
"            textmsg(" + QUOTATION + "jog_speed_vector_dt: " + QUOTATION + ", jog_speed_vector_dt)" + NEW_LINE +
"            textmsg(" + QUOTATION + "jog_feature: " + QUOTATION + ", jog_feature)" + NEW_LINE +
"            textmsg(" + QUOTATION + "start_jog done" + QUOTATION + ")    " + NEW_LINE +
"        elif cmd == 42:" + NEW_LINE +
"            textmsg(" + QUOTATION + "stop_jog" + QUOTATION + ")" + NEW_LINE +
"            lock.acquire()" + NEW_LINE +
"            if jog_thrd != 0:" + NEW_LINE +
"                textmsg(" + QUOTATION + "stopping jogging - killing jog_thrd" + QUOTATION + ")" + NEW_LINE +
"                stop_thread(jog_thrd)" + NEW_LINE +
"                jog_thrd = 0" + NEW_LINE +
"            end" + NEW_LINE +
"            lock.release()" + NEW_LINE +
"            stopl(1.2)" + NEW_LINE +
"            textmsg(" + QUOTATION + "stop_jog done" + QUOTATION + ")" + NEW_LINE +
"        elif cmd == 43:" + NEW_LINE +
"            textmsg(" + QUOTATION + "get_forward_kinematics_default" + QUOTATION + ")" + NEW_LINE +
"            forward_kin = get_forward_kin()" + NEW_LINE +
"            textmsg(forward_kin)" + NEW_LINE +
"            pose_to_output_float_registers(0, forward_kin)" + NEW_LINE +
"            textmsg(" + QUOTATION + "get_forward_kinematics_default done" + QUOTATION + ")" + NEW_LINE +
"        elif cmd == 44:" + NEW_LINE +
"            textmsg(" + QUOTATION + "get_forward_kinematics_args" + QUOTATION + ")" + NEW_LINE +
"            q = q_from_input_float_registers(0)" + NEW_LINE +
"            tcp_offset = pose_from_input_float_registers(6)" + NEW_LINE +
"            forward_kin = get_forward_kin(q, tcp_offset)" + NEW_LINE +
"            textmsg(forward_kin)" + NEW_LINE +
"            pose_to_output_float_registers(0, forward_kin)" + NEW_LINE +
"            textmsg(" + QUOTATION + "get_forward_kinematics_args done" + QUOTATION + ")" + NEW_LINE +
"        elif cmd == 45:" + NEW_LINE +
"            textmsg(" + QUOTATION + "move_path" + QUOTATION + ")" + NEW_LINE +
"            async = read_input_integer_reg(1)" + NEW_LINE +
"            textmsg(" + QUOTATION + "async: " + QUOTATION + ", async)" + NEW_LINE +
"            stop_async_move()" + NEW_LINE +
"            if async == 1:" + NEW_LINE +
"                lock.acquire()" + NEW_LINE +
"                move_type = 4 # move_path" + NEW_LINE +
"                lock.release()" + NEW_LINE +
"                move_thrd = start_thread(move_thread,())" + NEW_LINE +
"            else:" + NEW_LINE +
"                exec_move_path()" + NEW_LINE +
"                textmsg(" + QUOTATION + "move_path done" + QUOTATION + ")" + NEW_LINE +
"            end" + NEW_LINE +
"        elif cmd == 46:" + NEW_LINE +
"            textmsg(" + QUOTATION + "get_inverse_kin_default" + QUOTATION + ")" + NEW_LINE +
"            x = pose_from_input_float_registers(0)" + NEW_LINE +
"            textmsg(" + QUOTATION + "target pose:" + QUOTATION + ",x)" + NEW_LINE +
"            q = get_inverse_kin(x)" + NEW_LINE +
"            q_to_output_float_registers(0, q)" + NEW_LINE +
"            textmsg(" + QUOTATION + "inverse q:" + QUOTATION + ",q)" + NEW_LINE +
"            textmsg(" + QUOTATION + "get_inverse_kin_default done" + QUOTATION + ")" + NEW_LINE +
"        elif cmd == 60:" + NEW_LINE +
"            # get_sensor_force" + NEW_LINE +
"            sensor_force = get_tcp_force()" + NEW_LINE +
"            q_to_output_float_registers(0, sensor_force)" + NEW_LINE +
"        elif cmd == 61:" + NEW_LINE +
"            # get_tcp_mas_cog" + NEW_LINE +
"            mas_cog = get_tool_mass_CoM(0)" + NEW_LINE +
"            mas_cog_output_float_registers(0, mas_cog)" + NEW_LINE +
//"        elif cmd == 47:" + NEW_LINE +
//"            textmsg(" + QUOTATION + "is_steady" + QUOTATION + ")" + NEW_LINE +
//"            robot_is_steady = is_steady()" + NEW_LINE +
//"            if robot_is_steady == True:" + NEW_LINE +
//"                write_output_integer_reg(1, 1)" + NEW_LINE +
//"            else:" + NEW_LINE +
//"                write_output_integer_reg(1, 0)" + NEW_LINE +
////"              end" + NEW_LINE +
//"            textmsg(" + QUOTATION + "is_steady done" + QUOTATION + ")" + NEW_LINE +
//"        elif cmd == 51:" + NEW_LINE +
//"              textmsg(" + QUOTATION + "move_until_contact" + QUOTATION + ")" + NEW_LINE +
//"$V54          xd = q_from_input_float_registers(0)" + NEW_LINE +
//"$V54          contact_dir = pose_from_input_float_registers(6)" + NEW_LINE +
//"$V54          acc = read_input_float_reg(12)" + NEW_LINE +
//"$V54          while True:" + NEW_LINE +
//"$V54             step_back = tool_contact(direction=contact_dir)" + NEW_LINE +
//"$V54             if step_back <= 0:" + NEW_LINE +
//"$V54                # Continue moving with specified speed vector" + NEW_LINE +
//"$V54                speedl(xd, acc, t=get_steptime())" + NEW_LINE +
//"$V54             else:" + NEW_LINE +
//"$V54                # Contact detected!" + NEW_LINE +
//"$V54                # Get q for when the contact was first seen" + NEW_LINE +
//"$V54                q = get_actual_joint_positions_history(step_back)" + NEW_LINE +
//"$V54                # Stop the movement" + NEW_LINE +
//"$V54                stopl(3)" + NEW_LINE +
//"$V54                # Move to the initial contact point" + NEW_LINE +
//"$V54                movel(q)" + NEW_LINE +
//"$V54                break" + NEW_LINE +
//"$V54             end" + NEW_LINE +
//"$V54          end" + NEW_LINE +
//"              textmsg(" + QUOTATION + "move_until_contact done" + QUOTATION + ")" + NEW_LINE +
//"" + NEW_LINE +
"        elif cmd == 255:" + NEW_LINE +
"            textmsg(" + QUOTATION + "Received stop script!" + QUOTATION + ")" + NEW_LINE +
"        end" + NEW_LINE +
"" + NEW_LINE +
"        if cmd != 255:" + NEW_LINE +
"            signal_done_with_cmd()" + NEW_LINE +
"        end" + NEW_LINE +
"" + NEW_LINE +
"        return cmd != 255" + NEW_LINE +
"    end" + NEW_LINE +
"" + NEW_LINE +
"# HEADER_END" + NEW_LINE +
"" + NEW_LINE +
"# NODE_CONTROL_LOOP_BEGINS" + NEW_LINE +
"" + NEW_LINE +
"    ###################################" + NEW_LINE +
"    # RTSI Control script - Main loop #" + NEW_LINE +
"    ###################################" + NEW_LINE +
"" + NEW_LINE +
"    textmsg(" + QUOTATION + "RTSI Control Script Loaded (14.04.2023-12:30)" + QUOTATION + ")" + NEW_LINE +
"" + NEW_LINE +
"    # Initialize gain and damping for force mode to a more stable default" + NEW_LINE +
//"$V50force_mode_set_gain_scaling(0.5)" + NEW_LINE +
//"$V35force_mode_set_damping(0.025)" + NEW_LINE +
"" + NEW_LINE +
"    keep_running = True" + NEW_LINE +
"    executing_cmd = False" + NEW_LINE +
"    reset_async_progress()" + NEW_LINE +
"    signal_ready()" + NEW_LINE +
"" + NEW_LINE +
"    while keep_running:" + NEW_LINE +
"        cmd = rtde_cmd()" + NEW_LINE +
"        if cmd == 24 or cmd == 11 or cmd == 9 or cmd == 10 or cmd == 6 or cmd == 25 or cmd == 26 or cmd == 27 or cmd == 38:" + NEW_LINE +
"            # for realtime commands simply process and signal ready." + NEW_LINE +
"            keep_running = process_cmd()" + NEW_LINE +
"            signal_ready()" + NEW_LINE +
"        else:" + NEW_LINE +
"            # regular mode" + NEW_LINE +
"            if cmd == 0:" + NEW_LINE +
"                executing_cmd = False" + NEW_LINE +
"                signal_ready()" + NEW_LINE +
"            else:" + NEW_LINE +
"                if not executing_cmd:" + NEW_LINE +
"                    keep_running = process_cmd()" + NEW_LINE +
"                end" + NEW_LINE +
"                executing_cmd = True" + NEW_LINE +
"            end" + NEW_LINE +
"        end" + NEW_LINE +
"" + NEW_LINE +
"        sync()" + NEW_LINE +
"    end" + NEW_LINE +
"    textmsg(" + QUOTATION + "RTSI Control Script Terminated" + QUOTATION + ")" + NEW_LINE +
"# NODE_CONTROL_LOOP_ENDS" + NEW_LINE +
"";