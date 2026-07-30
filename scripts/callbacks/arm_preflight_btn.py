# Promoted onto wenyi::robot_arm. Delegates to the nested CSV asset, which
# owns the Python module and whose input is the POSE_SOURCE switch, so the
# check runs against whatever pose is selected.
_n = kwargs["node"].node("robot_csv_io")
if _n is None:
    raise hou.NodeError("robot_csv_io is missing from inside this asset.")
exec(_n.type().definition().sections()["PythonModule"].contents())
preflight({"node": _n})
