/**
 * @file bag_decoder.hpp
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief CsvWriter for csv file.
 *
 * Courtesy to p-ranav/csv2, modified from
 * https://github.com/p-ranav/csv2/blob/master/include/csv2/writer.hpp.
 */

#ifndef CSV_WRITER_HPP
#define CSV_WRITER_HPP

// stl include
#include <fstream>
#include <iomanip>
#include <iterator>
#include <string>
#include <utility>

class CsvWriter {
 public:
  /// @brief Constructor of CsvWriter.
  CsvWriter(const std::string &file) {
    stream_.open(file, std::ios::out | std::ios::trunc);
    stream_ << std::fixed << std::setprecision(2);
  }

  /// @brief Destructor of CsvWriter.
  ~CsvWriter() { stream_.close(); }

  /**
   * @brief Get the stream to write to the csv file.
   *
   * @return std::ofstream& Stream to write to the csv file.
   */
  std::ofstream &get_stream() { return stream_; }

  /**
   * @brief Write a single row to csv file.
   *
   * @tparam Container Type of container, it should support begin() and end()
   * iterator and its element should support << operator.
   * @param row Container of values.
   */
  template <typename Container>
  void write_row(Container &&row) {
    const auto &strings = std::forward<Container>(row);

    std::copy(strings.begin(), strings.end() - 1,
              std::ostream_iterator<
                  typename std::remove_reference_t<Container>::value_type>(
                  stream_, ","));
    stream_ << strings.back() << "\n";
  }

  /**
   * @brief Write multiple rows to csv file.
   *
   * @tparam Container Type of container, it should support begin() and end()
   * iterator and its elements should also support begin() and end() iterator
   * and it's element should support << operator.
   * @param rows Container of rows.
   */
  template <typename Container>
  void write_rows(Container &&rows) {
    const auto &container_of_rows = std::forward<Container>(rows);
    for (const auto &row : container_of_rows) {
      write_row(row);
    }
  }

 private:
  /// @brief Ofstream for writing csv file.
  std::ofstream stream_;
};

#endif  // CSV_WRITER_HPP
