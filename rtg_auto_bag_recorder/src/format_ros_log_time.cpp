#include <ctime>
#include <iomanip>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>

int main()
{
  const std::regex timestamp_pattern(R"(\[(\d{10})\.(\d{9})\])");
  const std::regex local_timestamp_pattern(
    R"(\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{9}) \+0800\])");
  const std::regex message_time_pattern(
    R"((当前时间=\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{9}) \+0800)");
  std::string line;
  while (std::getline(std::cin, line)) {
    std::string formatted;
    std::size_t previous_end = 0;
    for (auto match = std::sregex_iterator(line.begin(), line.end(), timestamp_pattern);
      match != std::sregex_iterator(); ++match)
    {
      formatted.append(line, previous_end, static_cast<std::size_t>(match->position()) - previous_end);
      const std::time_t seconds = std::stoll((*match)[1].str());
      std::tm local_time{};
      localtime_r(&seconds, &local_time);
      std::ostringstream date;
      date << '[' << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S")
           << '.' << (*match)[2].str() << ']';
      formatted += date.str();
      previous_end = static_cast<std::size_t>(match->position() + match->length());
    }
    formatted.append(line, previous_end, std::string::npos);
    formatted = std::regex_replace(formatted, local_timestamp_pattern, "[$1]");
    formatted = std::regex_replace(formatted, message_time_pattern, "$1");
    std::cout << formatted << std::endl;
  }
}
