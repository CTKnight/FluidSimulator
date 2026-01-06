#include "fluid/cli.h"

#include <sstream>
#include <unordered_map>

namespace fluid {
namespace cli {
namespace {

std::unordered_map<std::string, OptionSpec> build_spec_map(
    const std::vector<OptionSpec>& specs) {
  std::unordered_map<std::string, OptionSpec> map;
  for (const auto& spec : specs) {
    map.emplace(spec.name, spec);
  }
  return map;
}

}  // namespace

bool ParseResult::has(const std::string& name) const {
  return values.find(name) != values.end();
}

std::string ParseResult::value(const std::string& name,
                               const std::string& fallback) const {
  auto it = values.find(name);
  if (it == values.end()) {
    return fallback;
  }
  return it->second;
}

ParseResult parse_args(int argc,
                       char** argv,
                       const std::vector<OptionSpec>& specs) {
  ParseResult result;
  const auto spec_map = build_spec_map(specs);

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--") {
      for (int j = i + 1; j < argc; ++j) {
        result.positionals.push_back(argv[j]);
      }
      break;
    }
    if (arg == "-h" || arg == "--help") {
      result.values.emplace("help", "1");
      continue;
    }
    if (arg.rfind("--", 0) == 0) {
      const std::string opt = arg.substr(2);
      if (opt.empty()) {
        result.ok = false;
        result.error = "Invalid option: --";
        return result;
      }
      const auto eq_pos = opt.find('=');
      std::string name = opt;
      std::string value;
      if (eq_pos != std::string::npos) {
        name = opt.substr(0, eq_pos);
        value = opt.substr(eq_pos + 1);
      }
      auto it = spec_map.find(name);
      if (it == spec_map.end()) {
        result.ok = false;
        result.error = "Unknown option: --" + name;
        return result;
      }
      const OptionSpec& spec = it->second;
      if (spec.type == OptionType::Flag) {
        if (eq_pos != std::string::npos && !value.empty()) {
          result.ok = false;
          result.error = "Option does not take a value: --" + name;
          return result;
        }
        result.values[name] = "1";
        continue;
      }
      if (eq_pos == std::string::npos) {
        if (i + 1 >= argc) {
          result.ok = false;
          result.error = "Missing value for option: --" + name;
          return result;
        }
        value = argv[++i];
      }
      result.values[name] = value;
      continue;
    }
    result.positionals.push_back(arg);
  }

  return result;
}

std::string make_usage(const char* argv0, const std::vector<OptionSpec>& specs) {
  std::ostringstream out;
  out << "Usage: " << argv0 << " [options]" << '\n';
  out << '\n' << "Options:" << '\n';
  out << "  -h, --help" << '\n';
  for (const auto& spec : specs) {
    out << "  --" << spec.name;
    if (spec.type == OptionType::Value) {
      out << " <value>";
    }
    if (!spec.help.empty()) {
      out << "\n      " << spec.help;
    }
    out << '\n';
  }
  return out.str();
}

}  // namespace cli
}  // namespace fluid
