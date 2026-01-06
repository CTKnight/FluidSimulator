#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace fluid {
namespace cli {

enum class OptionType {
  Flag,
  Value,
};

struct OptionSpec {
  std::string name;
  OptionType type = OptionType::Flag;
  std::string help;
};

struct ParseResult {
  bool ok = true;
  std::string error;
  std::unordered_map<std::string, std::string> values;
  std::vector<std::string> positionals;

  bool has(const std::string& name) const;
  std::string value(const std::string& name,
                    const std::string& fallback = "") const;
};

ParseResult parse_args(int argc,
                       char** argv,
                       const std::vector<OptionSpec>& specs);

std::string make_usage(const char* argv0, const std::vector<OptionSpec>& specs);

}  // namespace cli
}  // namespace fluid
