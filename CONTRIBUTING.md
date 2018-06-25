Coding conventions
=================

Formatting
----------

- **Length** of a line should not be longer than **120** characters. In the rare
  cases when we need a longer line (e.g. in a preprocessor macro) use backslash
  character to mark a continuation.
- **Tab characters** are **not** welcome in the source. Replace them with
  spaces.
- **Indentation** is 4 characters. Contininuation indent is also 4 charachters.
- **Parameter list** is a frequent place, when a line tends to be longer than
  the limit. Break the line and align parameters, as seen in the example below.

  ```cpp
  void aMethodWithManyParameters(
      const std::string& str1,
      const std::string& str2,
      const std::int32_t id);
  ```
- **Namespaces** are not indented.

  ```cpp
  namespace railroad
  {
  /* ... */
  } // railroad
  ```
- **Blocks** should follow the K&R style, where related opening and closing 
  brackets for namespaces, classes and function blocks should be placed to the 
  same column; while other compound statements should have their opening braces 
  at the same line as their respective control statements.
- **Class declarations** should use only one `public`, `protected` and
  `private` part (in this order). The keywords `public`, `protected`,
  `private` are not indented, the member declarations are indented as usual
  (with 4 spaces). Inside a visibility class declare types first.

  ```cpp
  class MyClass
  {
  public:
      int getX();

  protected:
      int _protX;

  private:
      int _privX;
  };
  ```
- **Friend** declarations, if any, should be placed before the public
  visibility items, before the public keyword.
- The pointer and reference qualifier `*` and `&` letters should come
  **directly** after the type, followed by a space and then the variable's name:
  `int* ptr`, `const MyType& rhs`, `std::unique_ptr<T>&& data`.

Naming
------

- **File names** should contain ASCII characters and written in upper CamelCase 
  (with a few exceptions like `main.cpp`). Avoid other characters, like dash (-).
  Header file extension is `.h`, source file extension is `.cpp`.
- **Class and Type names** are written in upper CamelCase. Avoid underscore in class
  or type names. Pointers to major types should be typedef-ed, and should be
  called according the pointed type with a `Ptr` suffix.
- **Function names** start with lowercase letter, and have a capital letter for
  each new major tag. We do not require different names for static methods, or
  global functions.
- **Class member names** with private or protected visibility start with 
  underscore character following a lowercase letter, and have a capital letter 
  for each new major tag. Do not use other underscores in member names.
- **Function parameter names** start with a lowercase letter, and have a capital 
  letter for each new major tag. Do not use other underscores in parameter names.
- **Namespace names** are written in lower case. As for now, a single namespace 
  `railroad` is used only.

Headers
-------

- **Include guards** are mandatory for all headers. The guard names are all
  capital letters, and should start with the `RAILROAD_` prefix. For example
  the `AboveFilter.h` has the following header guard: `RAILROAD_ABOVEFILTER_H`.

  ```cpp
  #ifndef RAILROAD_ABOVEFILTER_H
  #define RAILROAD_ABOVEFILTER_H

  namespace railroad
  {

  class AboveFilter : public CloudProcessor
  {
  /*...*/
  };

  } // railroad

  #endif //RAILROAD_ABOVEFILTER_H
  ```
- **Order of the inclusion of headers** - either in source files or in other
  header files - should be the following: First include standard C++ headers,
  then Boost headers, then other supporting library headers (LASlib, PCL, OpenCV, etc.), 
  then your implementing headers.

  ```cpp
  #include <vector>
  #include <map>

  #include <boost/program_options.hpp>
  #include <boost/filesystem.hpp>

  #include <lasreader.hpp>
  #include <laswriter.hpp>

  #include <pcl/point_types.h>
  #include <pcl/point_cloud.h>

  #include <opencv2/highgui/highgui.hpp>
  #include <opencv2/imgproc/imgproc.hpp>

  #include "helpers/LogHelper.h"
  #include "helpers/LASHelper.h"

  #include "filters/DensityFilter.h"
  #include "filters/AboveFilter.h"
  ```
- **Never** apply `using namespace` directive in headers.
