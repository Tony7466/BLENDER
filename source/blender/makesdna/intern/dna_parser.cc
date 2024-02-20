/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/spirit/home/x3.hpp>
#include <iostream>
#include <optional>
#include <variant>

#include "dna_parser.hh"

BOOST_FUSION_ADAPT_STRUCT(blender::dna::parser::ast::ConstInt, name, value);
BOOST_FUSION_ADAPT_STRUCT(blender::dna::parser::ast::Variable::Item, ptr, name, size);
BOOST_FUSION_ADAPT_STRUCT(blender::dna::parser::ast::Variable, const_tag, type, items);
BOOST_FUSION_ADAPT_STRUCT(blender::dna::parser::ast::Struct, name, items);
BOOST_FUSION_ADAPT_STRUCT(blender::dna::parser::ast::FunctionPtr, type, name, params);
BOOST_FUSION_ADAPT_STRUCT(blender::dna::parser::ast::Enum::Item, name, value);
BOOST_FUSION_ADAPT_STRUCT(blender::dna::parser::ast::Enum, name, type, items);

/** Required traits so `blender::Vector` could be used by `x3`. */
namespace boost::spirit::x3::traits {
template<typename T> struct push_back_container<blender::Vector<T>> {
  static bool call(blender::Vector<T> &c, T &&val)
  {
    c.append(std::move(val));
    return true;
  }
  static bool call(blender::Vector<T> &c, blender::dna::parser::ast::Omitted &&val)
  {
    return true;
  }
};

template<typename T> struct is_empty_container<blender::Vector<T>> {
  static bool call(blender::Vector<T> const &c)
  {
    return c.is_empty();
  }
};
}  // namespace boost::spirit::x3::traits

namespace blender::dna::parser {

namespace x3 = boost::spirit::x3;

#define DEF_RULE(name, type) x3::rule<struct name, type> const name{#name};

DEF_RULE(const_int, ast::ConstInt);

DEF_RULE(identifier, std::string);
DEF_RULE(int_string, std::string);

DEF_RULE(variable_type, std::string);
DEF_RULE(variable_ptr, std::string);
DEF_RULE(variable_item, ast::Variable::Item);
DEF_RULE(variable, ast::Variable);

DEF_RULE(param, ast::Variable);
DEF_RULE(function_ptr, ast::FunctionPtr);
DEF_RULE(type_struct, ast::Struct);
DEF_RULE(enum_item_expr, std::string);
DEF_RULE(enum_item_value, ast::Enum::Item::Value);
DEF_RULE(enum_item, ast::Enum::Item);
DEF_RULE(type_enum, ast::Enum);
DEF_RULE(cpp, Vector<ast::CppType>);

/** Identifier is a combination of alphanumeric and underscore characters. */
auto identifier_def = x3::lexeme[(x3::alpha | x3::char_('_')) >> *(x3::alnum | x3::char_('_'))];

auto int_string_def = x3::lexeme[+x3::digit >> -x3::char_('u')];

/** Simple integer constant declaration. */
auto const_int_def = "#define" >> identifier >> x3::int32;

/* -------------------------------------------------------------------- */
/** \name Variable definition
 * \{ */

auto variable_type_def = -(x3::string("unsigned") | x3::lit("struct")) >> identifier;

auto variable_ptr_def = +x3::char_('*');

auto variable_size_def = ('[' >> (identifier | x3::int32) >> ']');

auto variable_item_def = -variable_ptr >> identifier >> *variable_size_def;

auto variable_def = x3::matches[x3::string("const")] >> variable_type >> (variable_item % ',');

/** \} */

/** Struct definition. */
auto type_struct_def = -x3::lit("typedef") >> "struct" >> identifier >> '{' >>
                       *((function_ptr | variable) >> ';') >> '}' >> -x3::omit[identifier] >> ";";

/* -------------------------------------------------------------------- */
/** \name Enum definition
 * \{ */

auto enum_item_expr_def = (identifier | int_string) >>
                          +((x3::string("<<") | x3::string("|")) >> (identifier | int_string));

auto enum_item_value_def = (enum_item_expr | identifier | x3::int32) |
                           ('(' >> (enum_item_expr | identifier | x3::int32) >> ')');

auto enum_item_def = identifier >> -('=' >> enum_item_value);

auto type_enum_def = -x3::lit("typedef") >> "enum" >> -identifier >> -(':' >> identifier) >> '{' >>
                     *(enum_item >> ',') >> "}" >> -x3::omit[identifier] >> ";";

/** \} */

/** Struct forward declaration. */
auto fw_declare_def = -x3::lit("typedef") >> "struct" >> (identifier % ',') >> ";";

/* -------------------------------------------------------------------- */
/** \name Function definition
 * \{ */

/** Function parameter, same as a variable. */
auto param_def = x3::matches[x3::string("const")] >> variable_type >> x3::repeat(1)[variable_item];

/** Function pointer definition. */
auto function_ptr_def = variable_type >> '(' >> '*' >> identifier >> ')' >> '(' >>
                        *(param % ',') >> ')';

/** \} */

auto cpp_def = *((const_int                                                      //
                  | type_enum                                                    //
                  | x3::omit[(x3::char_('#') >> x3::char_('#') >> type_struct)]  //
                  | type_struct                                                  //
                  | x3::omit[fw_declare_def]                                     //
                  | (function_ptr >> ';')                                        //
                  | x3::omit[x3::lit("#pragma once")]                            //
                  ));

auto multiline_comment_def = "/*" >> *(x3::char_ - "*/") >> "*/";
auto line_comment_def = "//" >> *(x3::char_ - x3::eol) >> x3::eol;

auto define_def = ("#define" >> *(x3::char_ - (x3::eol | ('\\' >> x3::eol))) % ('\\' >> x3::eol)) -
                  (("#define ") >> identifier >> ' ' >> x3::int32);

auto ifdef_def = (x3::lit("#ifdef") | x3::lit("#ifndef") | x3::lit("#if")) >>
                 *(x3::char_ - "#endif") >> "#endif";

auto dna_deprecated_def = x3::lit("DNA_DEPRECATED");
auto dna_define_cxx_def = "DNA_DEFINE_CXX_METHODS" >> *~x3::char_(")") >> x3::lit(')');
auto enum_operators_def = "ENUM_OPERATORS" >> *(~x3::char_(")")) >> x3::lit(')') >> -x3::lit(';');
auto include_def = x3::lit("#include") >> *(x3::char_ - x3::eol);

auto skippers = multiline_comment_def  //
                | line_comment_def     //
                | ifdef_def            //
                | define_def           //
                | dna_deprecated_def   //
                | dna_define_cxx_def   //
                | include_def          //
                | enum_operators_def   //
    ;

BOOST_SPIRIT_DEFINE(const_int);
BOOST_SPIRIT_DEFINE(int_string);
BOOST_SPIRIT_DEFINE(identifier);
BOOST_SPIRIT_DEFINE(variable_ptr);
BOOST_SPIRIT_DEFINE(variable_type);
BOOST_SPIRIT_DEFINE(variable_item);
BOOST_SPIRIT_DEFINE(variable);
BOOST_SPIRIT_DEFINE(param);
BOOST_SPIRIT_DEFINE(function_ptr);
BOOST_SPIRIT_DEFINE(type_struct);
BOOST_SPIRIT_DEFINE(enum_item_expr);
BOOST_SPIRIT_DEFINE(enum_item_value);
BOOST_SPIRIT_DEFINE(enum_item);
BOOST_SPIRIT_DEFINE(type_enum);
BOOST_SPIRIT_DEFINE(cpp);

#ifdef DEBUG_PRINT_DNA_PARSER
struct StructMemberPrinter {

  void operator()(ast::Variable &var) const
  {
    if (var.const_tag) {
      std::cout << "const " << var.type << " ";
    }
    else {
      std::cout << var.type << " ";
    }
    bool first = true;
    for (auto &variable_item : var.items) {
      if (!first) {
        std::cout << ",";
      }
      first = false;
      std::cout << variable_item.ptr.value_or("") << variable_item.name;
      for (auto &size : variable_item.size) {
        if (std::holds_alternative<std::string>(size)) {
          std::cout << "[" << std::get<std::string>(size) << "]";
        }
        else {
          std::cout << "[" << std::get<int32_t>(size) << "]";
        }
      }
    }
  }
  void operator()(ast::FunctionPtr &fn) const
  {
    std::cout << fn.type << " (*" << fn.name << ")(";
    for (ast::Variable &var : fn.params) {
      if (var.const_tag) {
        std::cout << "const " << var.type << " ";
      }
      else {
        std::cout << var.type << " ";
      }
      const bool is_last = &fn.params.last() != &var;

      std::cout << var.items[0].ptr.value_or("") << var.items[0].name;
      for (auto &size : var.items[0].size) {
        if (std::holds_alternative<std::string>(size)) {
          std::cout << "[" << std::get<std::string>(size) << "]";
        }
        else {
          std::cout << "[" << std::get<int32_t>(size) << "]";
        }
      }
      if (is_last) {
        std::cout << ",";
      }
    }
    std::cout << ")";
  }
};

struct ParserDebugPrinter {
  void operator()(ast::Omitted & /*val*/) const
  {
    std::cout << "{}\n";
  }
  void operator()(ast::ConstInt &val) const
  {
    std::cout << "#define " << val.name << " " << val.value << "\n";
  }
  void operator()(ast::Enum &val) const
  {
    std::cout << "enum " << val.name.value_or("unnamed")
              << (val.type ? ": " + val.type.value() : "") << " {\n";
    for (auto &item : val.items) {
      std::cout << "   " << item.name;
      if (item.value) {
        if (std::holds_alternative<std::string>(item.value.value())) {
          std::cout << " = " << std::get<std::string>(item.value.value());
        }
        else {
          std::cout << " = " << std::get<int32_t>(item.value.value());
        }
      }
      std::cout << ",\n";
    }
    std::cout << "};\n";
  }
  void operator()(ast::Struct &val) const
  {
    std::cout << "struct " << val.name << " {\n";
    for (auto &item : val.items) {
      std::cout << "    ";
      std::visit(StructMemberPrinter{}, item);
      std::cout << ";\n";
    }
    std::cout << "};\n";
  }
  void operator()(ast::FunctionPtr &fn) const
  {
    StructMemberPrinter().operator()(fn);
    std::cout << "\n";
  }
};
#endif

bool parse_include(std::string_view text, std::string_view file, Vector<ast::CppType> &c)
{
  auto itr = text.begin();
  auto end = text.end();

  bool result = phrase_parse(itr, end, cpp, skippers | x3::space, c);
  if (itr != end) {
    int line = 0;
    int chart = 0;
    for (auto test_itr = text.begin(); test_itr < itr; test_itr++) {
      if (test_itr[0] == '\n') {
        line++;
        chart = 0;
      }
      chart++;
    }
    std::cout << file << "(" << line << "," << chart << ")\n";
    return false;
  }
#ifdef DEBUG_PRINT_DNA_PARSER
  for (auto &item : c) {
    std::visit(ParserDebugPrinter{}, item);
  }
#endif
  return true;
}
}  // namespace blender::dna::parser
