# Source this in every terminal before running anything ROS:  . ~/ros2_ws/env.sh
#
# WHY. ~/.profile activates venv-ardupilot, whose python3 (numpy 2.4.4) lands
# ahead of the system one (numpy 1.26.4) on PATH. `ros2 launch` starts
# python-shebang nodes that inherit that PATH, so they run under the wrong
# interpreter. Most nodes tolerate it; cv_bridge does not -- its compiled
# extension was built against numpy 1.x and raises at import:
#
#   A module that was compiled using NumPy 1.x cannot be run in NumPy 2.4.4
#   ... from cv_bridge.boost.cv_bridge_boost import cvtColorForDisplay
#
# which kills both image_enhancement_node instances a few seconds into the
# run while everything else keeps going -- so the launch looks healthy unless
# you read the log. Confirmed in logs/launch_anchorfree_20261008_020628.log
# (18 errors) against logs/launch_20261003_233919.log (clean).
#
# Same fix as run_trial.py's ROS_SETUP; this just makes it reusable
# interactively.
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v venv-ardupilot | tr '\n' ':')
unset VIRTUAL_ENV PYTHONHOME
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
