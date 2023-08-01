import sys
from pathlib import Path

directory = Path(__file__).parent / "nodes"
output_cc_file = Path(sys.argv[1])

output_cc_lines = []

output_cc_lines.append("#include \"NOD_register.hh\"")
output_cc_lines.append("#include \"node_geometry_register.hh\"")
output_cc_lines.append("void register_geometry_nodes()")
output_cc_lines.append("{")

for path in directory.glob("*.cc"):
    with open(path) as f:
        code = f.read()
    index = code.find("  register_node_type_geo_")
    if index == -1:
        continue
    function_name = code[index:code.find("(", index)]
    output_cc_lines.append(f"{function_name}();")

output_cc_lines.append("}")

with open(output_cc_file, "w") as f:
    f.write("\n".join(output_cc_lines))
