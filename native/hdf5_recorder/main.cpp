// Exclusive streaming disk owner. Local inherited pipes only; no robotics SDKs.
// Protocol v1: LE u32 frame size, opcode, bounded column/attribute blocks.
#include <H5Cpp.h>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr uint32_t kMaxFrame = 64 * 1024 * 1024;
struct Reader {
  const std::vector<char>& data;
  size_t pos = 0;
  const char* bytes(size_t size) {
    if (size > data.size() - pos) throw std::runtime_error("truncated block");
    auto p = data.data() + pos; pos += size; return p;
  }
  uint32_t number() {
    uint32_t n; std::memcpy(&n, bytes(4), 4); return n;
  }
  std::string text() {
    auto n = number(); auto p = bytes(n);
    if (std::memchr(p, 0, n)) throw std::runtime_error("NUL in string");
    return {p, n};
  }
};

struct Column {
  H5::DataSet dataset;
  H5::DataType type;
  std::vector<hsize_t> dims;
  explicit Column(H5::H5File& file, const std::string& path)
      : dataset(file.openDataSet(path)), type(dataset.getDataType()) {
    auto space = dataset.getSpace();
    int rank = space.getSimpleExtentNdims();
    if (rank < 1 || rank > 4) throw std::runtime_error("unsupported column rank");
    dims.resize(rank); space.getSimpleExtentDims(dims.data());
  }
  void append(Reader& in) {
    auto rows = in.number();
    if (!rows || rows > 4096) throw std::runtime_error("invalid block row count");
    auto count = dims; count[0] = rows;
    size_t elements = rows;
    for (size_t i = 1; i < dims.size(); ++i) {
      if (!dims[i] || elements > kMaxFrame / dims[i]) throw std::runtime_error("oversized shape");
      elements *= dims[i];
    }
    const void* values = nullptr;
    std::vector<std::string> strings;
    std::vector<const char*> pointers;
    std::vector<hvl_t> variable;
    if (type.getClass() == H5T_STRING && H5Tis_variable_str(type.getId()) > 0) {
      strings.reserve(elements); pointers.reserve(elements);
      for (size_t i = 0; i < elements; ++i) strings.push_back(in.text());
      for (const auto& s : strings) pointers.push_back(s.c_str());
      values = pointers.data();
    } else if (type.getClass() == H5T_VLEN) {
      H5::DataType base(H5Tget_super(type.getId()));
      auto width = base.getSize();
      variable.reserve(elements);
      for (size_t i = 0; i < elements; ++i) {
        auto length = in.number();
        if (length > kMaxFrame / width) throw std::runtime_error("oversized vlen");
        variable.push_back({length, const_cast<char*>(in.bytes(length * width))});
      }
      values = variable.data();
    } else {
      if (elements > kMaxFrame / type.getSize()) throw std::runtime_error("oversized numeric column");
      values = in.bytes(elements * type.getSize());
    }
    auto start = dims; for (size_t i = 1; i < dims.size(); ++i) start[i] = 0;
    if (dims[0] > std::numeric_limits<hsize_t>::max() - rows) throw std::runtime_error("row overflow");
    dims[0] += rows;
    dataset.extend(dims.data());
    auto space = dataset.getSpace();
    space.selectHyperslab(H5S_SELECT_SET, count.data(), start.data());
    H5::DataSpace memory(static_cast<int>(count.size()), count.data());
    dataset.write(values, type, memory, space);
  }
};

void complete(H5::H5File& file) {
  auto attr = file.openAttribute("complete");
  uint8_t value = 1;
  attr.write(attr.getDataType(), &value);
}
}

int main(int argc, char** argv) {
  H5::Exception::dontPrint();
  try {
    uint32_t endian = 1;
    if (argc != 2 || *reinterpret_cast<char*>(&endian) != 1)
      throw std::runtime_error("expected initialized file path on little-endian host");
    H5::H5File file(argv[1], H5F_ACC_RDWR);
    auto attr = file.openAttribute("complete"); uint8_t value = 1;
    attr.read(attr.getDataType(), &value);
    if (value) throw std::runtime_error("refusing completed file");
    std::map<std::string, std::unique_ptr<Column>> columns;
    std::cout << "READY 1\n" << std::flush;
    while (true) {
      uint32_t length = 0;
      std::cin.read(reinterpret_cast<char*>(&length), 4);
      if (!std::cin) throw std::runtime_error("recording pipe closed without finalization");
      if (!length || length > kMaxFrame) throw std::runtime_error("invalid message size");
      std::vector<char> frame(length);
      if (!std::cin.read(frame.data(), length)) throw std::runtime_error("truncated message");
      Reader in{frame}; char op = *in.bytes(1);
      if (op == 'B') {
        auto count = in.number();
        if (count > 2048) throw std::runtime_error("too many columns");
        for (uint32_t i = 0; i < count; ++i) {
          auto path = in.text();
          if (path.empty() || path[0] != '/') throw std::runtime_error("invalid dataset path");
          auto& column = columns[path];
          if (!column) column = std::make_unique<Column>(file, path);
          column->append(in);
        }
        auto attrs = in.number();
        if (attrs > 2048) throw std::runtime_error("too many attributes");
        for (uint32_t i = 0; i < attrs; ++i) {
          auto path = in.text(), name = in.text(), text = in.text();
          // Only the canonical joint-name metadata may change during capture.
          if (name != "names" && name != "joint_names" && name != "logical_id")
            throw std::runtime_error("unsupported runtime attribute");
          auto group = file.openGroup(path);
          H5::StrType type(H5::PredType::C_S1, H5T_VARIABLE); type.setCset(H5T_CSET_UTF8);
          if (group.attrExists(name)) group.removeAttr(name);
          auto a = group.createAttribute(name, type, H5::DataSpace(H5S_SCALAR));
          const char* p = text.c_str(); a.write(type, &p);
        }
      } else if (op != 'F' && op != 'C' && op != 'A') {
        throw std::runtime_error("unknown operation");
      }
      if (in.pos != frame.size()) throw std::runtime_error("trailing protocol bytes");
      if (op != 'B') file.flush(H5F_SCOPE_GLOBAL);
      if (op == 'C') { complete(file); file.flush(H5F_SCOPE_GLOBAL); }
      if (op == 'C' || op == 'A') {
        columns.clear(); attr.close(); file.close();
        std::cout << "OK\n" << std::flush; return 0;
      }
      std::cout << "OK\n" << std::flush;
    }
  } catch (const H5::Exception& e) {
    std::cerr << "native HDF5 failure: " << e.getDetailMsg() << '\n';
  } catch (const std::exception& e) {
    std::cerr << "native recorder failure: " << e.what() << '\n';
  }
  std::cout << "ERROR\n" << std::flush;
  return 1;
}
