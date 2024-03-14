"""
clear && python3 ~/blender-git/blender/source/blender/gpu/vulkan/scripts/vk_render_graph_gen.py
"""

import xml.etree.ElementTree as ET


### Utils - Formatting ###
def to_lower_snake_case(string):
    result = ""
    for char in string:
        if char.isupper() and len(result) != 0:
            result += "_"
        result += char.lower()
    return result


### Commands ###
def extract_type_names(commands, r_types):
    for command in commands:
        for param in command.findall("param"):
            param_type = param.findtext("type")
            if param_type not in r_types:
                r_types.append(param_type)


### Enumerations ###
def generate_enum_to_string_hpp(enum):
    vk_name = enum.get("name")
    result = ""
    result += f"const char *to_string({vk_name} {to_lower_snake_case(vk_name)});\n"
    return result


def generate_enum_to_string_cpp_case(elem):
    result = ""
    vk_elem_name = elem.get("name")
    result += f"    case {vk_elem_name}:\n"
    result += f"      return STRINGIFY({vk_elem_name});\n\n"
    return result


def generate_enum_to_string_cpp(enum, features, extensions):
    vk_name = enum.get("name")
    vk_name_parameter = to_lower_snake_case(vk_name)
    result = ""
    result += f"const char *to_string(const {vk_name} {vk_name_parameter})\n"
    result += "{\n"
    result += f"  switch ({vk_name_parameter}) {{\n"
    for elem in enum.findall("enum"):
        result += generate_enum_to_string_cpp_case(elem)
    for feature in features:
        enum_extensions = feature.findall(f"require/enum[@extends='{vk_name}']")
        if not enum_extensions:
            continue
        feature_name = feature.get("name")
        result += f"    /* Extensions for {feature_name}. */\n"
        for elem in enum_extensions:
            result += generate_enum_to_string_cpp_case(elem)
    for extension in extensions:
        enum_extensions = extension.findall(f"require/enum[@extends='{vk_name}']")
        if not enum_extensions:
            continue
        extension_name = extension.get("name")
        result += f"    /* Extensions for {extension_name}. */\n"
        for elem in enum_extensions:
            result += generate_enum_to_string_cpp_case(elem)

    result += "    default:\n"
    result += "      break;\n"

    result += "  }\n"
    result += f"  return STRINGIFY_ARG({vk_name_parameter});\n"
    result += "}\n"
    return result


### Bitflags ###
def generate_bitflag_to_string_hpp(vk_name):
    vk_name_parameter = to_lower_snake_case(vk_name)
    result = ""
    result += f"std::string to_string_{vk_name_parameter}({vk_name} {to_lower_snake_case(vk_name)});\n"
    return result


def generate_bitflag_to_string_cpp_case(vk_parameter_name, elem):
    vk_elem_name = elem.get("name")

    result = ""
    result += f"  if ({vk_parameter_name} & {vk_elem_name}) {{\n"
    result += f"    ss << STRINGIFY({vk_elem_name}) << \", \";\n"
    result += f"  }}\n"
    return result


def generate_bitflag_to_string_cpp(vk_name, enum, features, extensions):
    vk_enum_name = enum.get("name")
    vk_name_parameter = to_lower_snake_case(vk_name)
    result = ""
    result += f"std::string to_string_{vk_name_parameter}(const {vk_name} {vk_name_parameter})\n"
    result += "{\n"
    result += "  std::stringstream ss;\n"
    result += "\n"
    for elem in enum.findall("enum"):
        result += generate_bitflag_to_string_cpp_case(vk_name_parameter, elem)
    for feature in features:
        enum_extensions = feature.findall(f"require/enum[@extends='{vk_enum_name}']")
        if not enum_extensions:
            continue
        feature_name = feature.get("name")
        result += f"  /* Extensions for {feature_name}. */\n"
        for elem in enum_extensions:
            result += generate_bitflag_to_string_cpp_case(vk_name_parameter, elem)
    for extension in extensions:
        enum_extensions = extension.findall(f"require/enum[@extends='{vk_enum_name}']")
        if not enum_extensions:
            continue
        extension_name = extension.get("name")
        result += f"  /* Extensions for {extension_name}. */\n"
        for elem in enum_extensions:
            result += generate_bitflag_to_string_cpp_case(vk_name_parameter, elem)

    result += "\n"
    result += f"  std::string result = ss.str();\n"
    result += f"  if (result.size() >= 2) {{\n"
    result += f"    result.erase(result.size() - 2, 2);\n"
    result += f"  }}\n"
    result += f"  return result;\n"
    result += "}\n"
    return result


### Structs ###
def generate_struct_to_string_hpp(struct):
    vk_name = struct.get("name")
    vk_name_parameter = to_lower_snake_case(vk_name)
    result = ""
    result += f"std::string to_string(const {vk_name} &{vk_name_parameter}, int indentation_level=0);\n"
    return result


def generate_struct_to_string_cpp(struct):
    vk_name = struct.get("name")
    vk_name_parameter = to_lower_snake_case(vk_name)
    header = ""
    header += f"std::string to_string(const {vk_name} &{vk_name_parameter}, int indentation_level)\n"
    header += f"{{\n"
    result = ""
    result += f"  std::stringstream ss;\n"
    pre = ""
    indentation_used = False
    for member in struct.findall("member"):
        member_type = member.findtext("type")
        member_type_parameter = to_lower_snake_case(member_type)
        member_name = member.findtext("name")
        member_name_parameter = to_lower_snake_case(member_name)
        if member_name in MEMBERS_TO_IGNORE:
            continue

        result += f"  ss << \"{pre}{member_name_parameter}=\" << "
        if member_type in FLAGS_TO_GENERATE:
            result += f"to_string_{member_type_parameter}({vk_name_parameter}.{member_name})"
        elif member_type in ENUMS_TO_GENERATE:
            result += f"to_string({vk_name_parameter}.{member_name})"
        elif member_type in STRUCTS_TO_GENERATE:
            result += f"\"\\n\";\n"
            result += f"  ss << std::string(indentation_level * 2 + 2, ' ') << to_string({vk_name_parameter}.{member_name}, indentation_level + 1);\n"
            result += f"  ss << std::string(indentation_level * 2, ' ');"
            indentation_used = True
        else:
            result += f"{vk_name_parameter}.{member_name}"
        result += ";\n"
        pre = ", "
    result += f"\n"
    result += f"  return ss.str();\n"
    result += f"}}\n"
    if not indentation_used:
        header += "  UNUSED_VARS(indentation_level);\n"
    return header + result


# List of features that we use. These can extend our enum types and these extensions should also be parsed.
FEATURES = [
    "VK_VERSION_1_0",
    "VK_VERSION_1_1",
    "VK_VERSION_1_2",
]
EXTENSIONS = [
    "VK_KHR_swapchain",
]

COMMANDS_TO_GEN = [
    "vkCmdClearColorImage",
    "vkCmdClearDepthStencilImage",
    "vkCmdClearAttachments",

    "vkCmdCopyImageToBuffer",
    "vkCmdCopyBufferToImage",
    "vkCmdCopyImage",
    "vkCmdCopyBuffer",

    "vkCmdFillBuffer",
    "vkCmdBlitImage",

    "vkCmdBindDescriptorSets",
    "vkCmdPushConstants",
    "vkCmdBindIndexBuffer",
    "vkCmdBindVertexBuffers",
    "vkCmdBindPipeline",

    "vkCmdBeginRenderPass",
    "vkCmdEndRenderPass",
    "vkCmdDraw",
    "vkCmdDrawIndexed",
    "vkCmdDrawIndirect",
    "vkCmdDrawIndexedIndirect",

    "vkCmdDispatch",
    "vkCmdDispatchIndirect",

    "vkCmdPipelineBarrier",
]

ENUMS_TO_GENERATE = [
    "VkObjectType"
]
ENUMS_TO_IGNORE = [
    "VkStructureType"
]

FLAGS_TO_GENERATE = [
]

ALL_FLAGS = {
}
ALL_STRUCTS = []
MEMBERS_TO_IGNORE = [
    "sType", "pNext",
    # Disabled as these are arrays.
    "srcOffsets", "dstOffsets",
    # Disabled as it is an union
    "clearValue",
    # Disabled as we don't use cross queue synchronization
    "srcQueueFamilyIndex", "dstQueueFamilyIndex"
]


STRUCTS_TO_GENERATE = []

# VK_XML = "/home/jeroen/blender-git/blender/lib/linux_x64/vulkan/share/vulkan/registry/vk.xml"
VK_XML = "/Users/jeroen/blender-git/blender/lib/macos_arm64/vulkan/share/vulkan/registry/vk.xml"

tree = ET.parse(VK_XML)
root = tree.getroot()

# Find all commands
commands = []
for command in root.findall("commands/command"):
    command_name = command.findtext("proto/name")
    if command_name in COMMANDS_TO_GEN:
        commands.append(command)

for flag_type in root.findall("types/type[@category='bitmask']"):
    flag_type_name = flag_type.findtext("name")
    flag_type_bits_name = flag_type.get("requires")
    if flag_type_name and flag_type_bits_name:
        ALL_FLAGS[flag_type_name] = flag_type_bits_name


# - Extract types
types_undetermined = []
extract_type_names(commands, types_undetermined)
while types_undetermined:
    newly_found_types = []
    for type_name in types_undetermined:
        if root.find(f"enums[@name='{type_name}']"):
            if type_name not in ENUMS_TO_GENERATE and type_name not in ENUMS_TO_IGNORE:
                ENUMS_TO_GENERATE.append(type_name)
        elif type_name in ALL_FLAGS and type_name not in FLAGS_TO_GENERATE:
            FLAGS_TO_GENERATE.append(type_name)
        elif type_name not in STRUCTS_TO_GENERATE:
            struct = root.find(f"types/type[@category='struct'][@name='{type_name}']")
            if struct:
                STRUCTS_TO_GENERATE.append(type_name)
                for member in struct.findall("member/type"):
                    newly_found_types.append(member.text)

    types_undetermined = newly_found_types


ENUMS_TO_GENERATE.sort()
FLAGS_TO_GENERATE.sort()
STRUCTS_TO_GENERATE.sort()


vk_to_string_hpp = ""
vk_to_string_cpp = ""

# Find all features that we use.
features = []
for feature_name in FEATURES:
    feature = root.find(f"feature[@name='{feature_name}']")
    assert(feature)
    features.append(feature)
# Find all extensions that we use.
extensions = []
for extension_name in EXTENSIONS:
    extension = root.find(f"extensions/extension[@name='{extension_name}']")
    assert(extension)
    extensions.append(extension)

for enum_to_generate in ENUMS_TO_GENERATE:
    for enum in root.findall(f"enums[@name='{enum_to_generate}']"):
        vk_to_string_hpp += generate_enum_to_string_hpp(enum)
        vk_to_string_cpp += generate_enum_to_string_cpp(enum, features, extensions)
        vk_to_string_cpp += "\n"

for flag_to_generate in FLAGS_TO_GENERATE:
    enum_to_generate = ALL_FLAGS[flag_to_generate]
    for enum in root.findall(f"enums[@name='{enum_to_generate}']"):
        vk_to_string_hpp += generate_bitflag_to_string_hpp(flag_to_generate)
        vk_to_string_cpp += generate_bitflag_to_string_cpp(flag_to_generate, enum, features, extensions)
        vk_to_string_cpp += "\n"

for struct_to_generate in STRUCTS_TO_GENERATE:
    struct = root.find(f"types/type[@category='struct'][@name='{struct_to_generate}']")
    assert(struct)
    vk_to_string_hpp += generate_struct_to_string_hpp(struct)
    vk_to_string_cpp += generate_struct_to_string_cpp(struct)
    vk_to_string_cpp += "\n"

# print(vk_to_string_hpp)
print(vk_to_string_cpp)

