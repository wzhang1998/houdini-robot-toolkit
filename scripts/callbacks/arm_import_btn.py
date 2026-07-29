# Promoted onto wenyi::robot_arm. The work still belongs to the nested
# wenyi::robot_anim_csv_io asset: that asset owns the Python module, and its
# input is the POSE_SOURCE switch, so it already sees whichever pose is
# selected. Delegate to it rather than re-implement here.
#
# The original callback exec'd kwargs["node"]'s own PythonModule section. Once
# promoted, kwargs["node"] is the arm, which has no such section -- the button
# then did nothing at all, with no error.
_n = kwargs["node"].node("robot_csv_io")
if _n is None:
    raise hou.NodeError("robot_csv_io is missing from inside this asset.")
exec(_n.type().definition().sections()["PythonModule"].contents())
import_animation({"node": _n})
