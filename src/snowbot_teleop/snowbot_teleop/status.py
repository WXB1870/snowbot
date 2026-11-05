"""Operator view assembled from application and measured execution state."""

from bindu_runtime.common import seconds


def operator_view(app, feedback):
    view = dict(
        app.control.snapshot(app.now()),
        run_id=app.run_id,
        profile_hash=app.profile.digest,
        vr_source_id=app.control.source,
        source_id=(app.control.source + ":" + app.instance_id) if app.control.source else None,
        schema=1,
        device_backend=app.backend,
        coordinator_id=app.instance_id,
        stamp=app.now(),
        simulated=app.simulation,
        runtime_mode=app.runtime_mode,
        handoff_complete=app.handoff_complete,
        gripper_modes=list(feedback.modes),
        tool_phase=app.tool_phase,
        recording=app.recording,
        recording_enabled=app.recording_enabled,
        motion_limit=(
            app.motion_limit
            if app.control.phase == "following" and 0 <= app.now() - app.motion_limit_stamp <= 0.25
            else None
        ),
        physical_gripper_confirmed=None,
        actuator_feedback="joint_position",
        capabilities=dict(
            prepare=app.prepare_positions is not None,
            independent_arms=app.position_contract,
            mode_switch=app.position_contract,
            grippers=(
                bool(
                    app.position_ops.config["calibrated"]
                    or app.position_ops.config.get("allow_uncalibrated_direct", False)
                )
                if app.position_ops
                else True
            ),
            continuous_axial_grasp=getattr(app.control, "continuous_axial", False),
            axial_grasp=not getattr(app.control, "continuous_axial", False)
            and not app.unheld_test
            and (
                not app.operator_grasp and app.position_ops.axial_available
                if app.position_ops
                else True
            ),
        ),
        measured_grasp_policy="unheld_test" if app.unheld_test else "enforce",
        tool_grasp_confirmation=(
            "unheld_test" if app.unheld_test else "operator" if app.operator_grasp else "calibrated"
        ),
        direction_reference=getattr(app, "tool_cfg", {}).get("direction_reference", "fixed"),
        execution_state=app.state.state if app.state is not None else "unavailable",
        feedback_age_s=(
            app.now() - seconds(app.state.feedback_stamp) if app.state is not None else None
        ),
    )
    view.update(
        motion_stopped=feedback.healthy and feedback.stopped,
        tool_ground_policy=app.tool_cfg["demo"]["acceptance"].get("ground_policy", "required"),
        tool_variant="short_rod" if app.simulation_demo else "shovel",
        position_scale=app.tool_cfg["position_scale"],
        rotation_scale=1.0,
        mapping_frame="robot_base",
        control_point=(
            "individual_flange"
            if app.control.teleop_mode == "independent_arms"
            else (
                "grip_midpoint"
                if app.simulation_demo
                or app.tool_cfg.get("vr_tool_mapping") == "hand_line_midpoint"
                else "tool_origin"
            )
        ),
        orientation_source=(
            "individual_wrist"
            if app.control.teleop_mode == "independent_arms"
            else (
                "hand_line"
                if app.simulation_demo
                or app.tool_cfg.get("vr_tool_mapping") == "hand_line_midpoint"
                else "left_wrist"
            )
        ),
        model_joints=(
            dict(zip(app.state.joints.name, app.state.joints.position))
            if feedback.healthy
            else None
        ),
        input_max_age_s=app.control.max_age,
        stop_confirmed=(
            app.control.rest(app.now())
            and app.control.pending is None
            and app.running is None
            and app.handoff_complete
        ),
        grasp_applied=(
            None
            if app.control.teleop_mode == "independent_arms"
            else "right_axial" if app.control.virtual else "rigid"
        ),
    )
    if feedback.healthy and app.prepare_positions is not None:
        measured = view["model_joints"]
        view["measured_limit_joints"] = [
            n
            for n in app.names
            if n in measured
            and not app.profile.limits[n][0] <= measured[n] <= app.profile.limits[n][1]
        ]
        if view["measured_limit_joints"]:
            from snowbot_teleop.control import loading_guide

            view["operator_guide"] = loading_guide(view, feedback.modes)
    view.update(app.input_status(app.now()))
    view["stop_diagnostic"] = getattr(app, "stop_diagnostic", None)
    if app.base:
        view["base"] = app.base.snapshot()
    return view
