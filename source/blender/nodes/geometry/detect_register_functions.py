import sys
from pathlib import Path

directory = Path(__file__).parent / "nodes"
output_cc_file = Path(sys.argv[1])

include_lines = []
include_lines.append("#include \"NOD_register.hh\"")
include_lines.append("#include \"node_geometry_register.hh\"")

decl_lines = []

func_lines = []
func_lines.append("void register_geometry_nodes()")
func_lines.append("{")

for path in directory.glob("*.cc"):
    with open(path) as f:
        code = f.read()
    index = code.find("NOD_REGISTER_NODE(")
    if index == -1:
        continue
    function_name = code[index+len("NOD_REGISTER_NODE("):code.find(")", index)] + "_auto_run"
    func_lines.append(f"  {function_name}();")
    decl_lines.append(f"void {function_name}();")

func_lines.append("}")

with open(output_cc_file, "w") as f:
    f.write("\n".join(include_lines + decl_lines + func_lines))
