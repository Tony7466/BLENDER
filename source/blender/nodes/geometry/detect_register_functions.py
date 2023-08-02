import sys
from pathlib import Path
import re

directory = Path(__file__).parent / "nodes"
output_cc_file = Path(sys.argv[1])

include_lines = []

decl_lines = []

func_lines = []
func_lines.append("void register_geometry_nodes();")
func_lines.append("void register_geometry_nodes()")
func_lines.append("{")

for path in directory.glob("*.cc"):
    with open(path) as f:
        code = f.read()

    namespace_parts = []

    for match in re.finditer(r"(^namespace ([\w:]+) \{)|(^\}  // namespace ([\w:]+))|(NOD_REGISTER_NODE\((\w+)\))", code, flags=re.MULTILINE):
        if entered_namespace := match.group(2):
            namespace_parts += entered_namespace.split("::")
        elif exited_namespace := match.group(4):
            del namespace_parts[-len(exited_namespace.split("::")):]
        elif function_name := match.group(6):
            namespace_str = "::".join(namespace_parts)
            auto_run_name = function_name + "_auto_run"
            func_lines.append(f"  {namespace_str}::{auto_run_name}();")
            if namespace_str:
                decl_lines.append(f"namespace {namespace_str} {{")
            decl_lines.append(f"void {auto_run_name}();")
            if namespace_str:
                decl_lines.append(f"}}")

func_lines.append("}")

with open(output_cc_file, "w") as f:
    f.write("\n".join(include_lines + decl_lines + [""] + func_lines))
