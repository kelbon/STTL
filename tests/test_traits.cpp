
#include "type_algorithms.hpp"

constexpr int Fooo(sttl::string_literal<"hello">) {
  return 4;
}
constexpr int Fooo(sttl::string_literal<u8"I know, it will be UB if value == 42">, int val) {
  return val;
}
template<int64_t N>
constexpr bool Fooo(sttl::integer_literal<N>) {
  return true;
}
template<char... Cs>
consteval auto foo() {
  return sttl::pattern_matching<[](std::integral_constant<char, Cs>) { return Cs; }...>;
}

int main() {
  static_assert(foo<'a', 'b', 'c'>()(std::integral_constant<char, 'c'>{}) == 'c');
  constexpr auto j = -10000_i;
  int j_0 = j;
  constexpr int64_t j_3 = j;
  (void)j_0, void(j_3);
  constexpr auto la = "hello"_fixs + "world"_fixs;
  (void)la;
  static_assert(Fooo(-0_i) == true);
  static_assert(Fooo(+0_i) ==true);
  static_assert(Fooo("hello"_s) == 4);
  static_assert(Fooo(-+150_i) == true);
  static_assert(Fooo(-111_i) == true);
  static_assert(Fooo(u8"I know, it will be UB if value == 42"_s, 16) == 16);
  static_assert(std::is_same_v<sttl::types::sort<sttl::typevec<int8_t, int32_t, int16_t>>,
                               sttl::typevec<int8_t, int16_t, int32_t>>);
  static_assert(std::is_same_v<sttl::types::sort<sttl::typevec<>>, sttl::typevec<>>);
  constexpr auto is_array = []<typename T>(std::type_identity<T>) {
    return std::is_array_v<T>;
  };
  static_assert(std::is_same_v<sttl::filter<is_array, void, float, double[10], int, char[], bool[1]>,
                               sttl::typevec<double[10], char[], bool[1]>>);
  static_assert(std::is_same_v<sttl::filter<is_array>, sttl::typevec<>>);
  static_assert(std::is_same_v<sttl::types::filter<is_array, sttl::typevec<>>, sttl::typevec<>>);
  static_assert(std::is_same_v<sttl::filter<is_array, int, void, int[10], bool>, sttl::typevec<int[10]>>);
  static_assert(std::is_same_v<sttl::types::filter<is_array, sttl::typevec<int, void, int[10], bool>>,
                               sttl::typevec<int[10]>>);
  constexpr auto convertible_to_bool = []<typename T>(std::type_identity<T>) {
    return std::convertible_to<T, bool>;
  };
  static_assert(sttl::all_of<convertible_to_bool, int, float, bool>);
  static_assert(sttl::all_of<convertible_to_bool, int, float, bool>);
  static_assert(sttl::types::all_of<sttl::typevec<int, float, bool>, convertible_to_bool>);
  static_assert(!sttl::types::all_of<sttl::typevec<void, float, bool>, convertible_to_bool>);
  // behavior same as std::any/none/any of for empty ranges
  static_assert(sttl::types::all_of<sttl::typevec<>, convertible_to_bool>);
  static_assert(sttl::types::none_of<sttl::typevec<>, convertible_to_bool>);
  static_assert(!sttl::types::atleast_one_of<sttl::typevec<>, convertible_to_bool>);

  static_assert(sttl::contains<int, int>);
  static_assert(!sttl::contains<int>);
  static_assert(sttl::contains<void, int, float, void>);
  static_assert(!sttl::contains<void, int, float>);
  static_assert(!sttl::types::contains<void, sttl::typevec<int, float>>);

  static_assert(sttl::count_if<is_array, int, int[], int[10]> == 2);
  static_assert(sttl::count_if<is_array> == 0);
  static_assert(sttl::types::count_if<is_array, sttl::typevec<>> == 0);
  static_assert(sttl::types::count_if<is_array, sttl::typevec<int, float[10]>> == 1);
  static_assert(sttl::count<int, int, int, int> == 3);
  static_assert(sttl::count<void> == 0);
  static_assert(sttl::types::count<void, sttl::typevec<void, int>> == 1);

  static_assert(std::is_same_v<sttl::element_t<0, int, float, double>, int>);
  static_assert(std::is_same_v<sttl::try_element_t<3, int, float, double>, sttl::notype>);
  static_assert(std::is_same_v<sttl::types::try_element_t<3, sttl::typevec<int, float, double>>, sttl::notype>);
  static_assert(std::is_same_v<sttl::try_element_t<0, int, float, double>, int>);
  static_assert(std::is_same_v<sttl::types::try_element_t<0, sttl::typevec<int, float, double>>, int>);
  static_assert(std::is_same_v<sttl::element_t<0, void>, void>);
  static_assert(std::is_same_v<sttl::types::element_t<0, sttl::typevec<void>>, void>);

  using var = sttl::Enum<int, float, double, bool>;
  static_assert(sttl::contains<int, int, float>);
  static_assert(sttl::contains<void, int, float, void>);
  static_assert(sttl::contains<int, int>);
  static_assert(!sttl::contains<char, int, float>);
  static_assert(!sttl::contains<int>);
  constexpr auto xvar = var{0};
  constexpr auto yvar = var{std::type_identity<bool>{}};
  using my_enum = sttl::tagged_enum<sttl::enum_value<int, 4>, sttl::enum_value<float, 8>,
                                    sttl::enum_value<bool, 1>, sttl::enum_value<char, 16>>;
  static_assert(sttl::visit_enum(
      []<typename... Ts>(Ts...) {
        return std::is_same_v<sttl::typevec<typename Ts::type...>, sttl::typevec<int, char, bool>> &&
               std::is_same_v<std::index_sequence<Ts::value...>, std::index_sequence<4, 16, 1>>;
      },
      my_enum(0), my_enum(3), my_enum(2)));
  static_assert(sttl::visit_enum(
      []<typename... Ts>(Ts...) {
        return std::is_same_v<sttl::typevec<typename Ts::type...>, sttl::typevec<int, bool, int>>;
      },
      xvar, yvar, xvar));
  static_assert(sttl::visit_enum<xvar, yvar, xvar, xvar>([]<typename... Ts>(Ts...) {
                  return (sizeof(typename Ts::type) + ...);
                }) == (sizeof(bool) + 3 * sizeof(int)));
  constexpr auto _1 = []<sttl::one_of<int, float> T, sttl::not_a<double> U>(T, U) {
    return 0;
  };
  constexpr auto _2 = [](auto&&...) {  // default
    return 1;
  };
  static_assert(sttl::find_first<int, float, int, double> == 1);
  static_assert(sttl::find_first<int&, float, int&, double, int&> == 1);
  static_assert(sttl::find_first<void, int, double> == sttl::npos);
  static_assert(sttl::find_first<int, int, int> == 0);
  static_assert(sttl::find_first<int> == sttl::npos);
  static_assert(sttl::find_first<void, void> == 0);
  static_assert(sttl::find_first<int, char, unsigned> == sttl::npos);
  static_assert(sttl::find_last<void, void, void> == 1);
  static_assert(sttl::find_last<void, void> == 0);
  static_assert(sttl::find_last<const volatile void*, void*, const void, const volatile void*, int> == 2);
  static_assert(sttl::find_last<int&, int&, int&, int&, float> == 2);
  static_assert(sttl::find_last<int&, int, const int, float> == sttl::npos);
  static_assert(sttl::find_last<int> == sttl::npos);
  
  static_assert(sttl::instance_of<std::vector<int>, std::vector>);
  static_assert(!sttl::instance_of<int, std::vector>);

  static_assert(
      std::is_same_v<
          sttl::types::merge<std::vector<int>, sttl::typevec<int>, sttl::typevec<float>, sttl::typevec<>>,
          sttl::typevec<int, std::allocator<int>, int, float>>);

  static_assert(
      std::is_same_v<sttl::types::extract<std::vector<int>>, sttl::typevec<int, std::allocator<int>>>);
  static_assert(sttl::is_notype_v<sttl::types::extract<int>>);
  // do not work on msvc =( buggy(
  //  static_assert(std::is_same_v<sttl::types::pop_back<sttl::typevec<int, float>>, sttl::typevec<int>>);
  //  static_assert(std::is_same_v<sttl::types::pop_back<sttl::typevec<int>>, sttl::typevec<>>);
  //  static_assert(sttl::is_notype_v<sttl::types::pop_back<sttl::typevec<>>>);

  using misres = sttl::types::mismatch<sttl::typevec<int, float, double>, sttl::typevec<int, float, double, void>>;
  static_assert(misres::index == sttl::npos);
  static_assert(std::is_same_v<misres::first, misres::second>);
  static_assert(sttl::is_notype_v<misres::first>);
  using misres1 = sttl::types::mismatch<sttl::typevec<int, float, double>, sttl::typevec<int, int, double, void>>;
  static_assert(misres1::index == 1);
  static_assert(std::is_same_v<misres1::first, float>);
  static_assert(std::is_same_v<misres1::second, int>);

  static_assert(std::is_same_v<sttl::reverse<int, float, double>, sttl::typevec<double, float, int>>);
  static_assert(std::is_same_v<sttl::reverse<int>, sttl::typevec<int>>);
  static_assert(std::is_same_v<sttl::reverse<int, int, double>, sttl::typevec<double, int, int>>);
  static_assert(std::is_same_v<sttl::reverse<>, sttl::typevec<>>);

  static_assert(std::is_same_v<sttl::last_t<int, float, void>, void>);
  static_assert(std::is_same_v<sttl::types::last_t<sttl::typevec<int, float, void>>, void>);
  static_assert(std::is_same_v<sttl::last_t<>, sttl::notype>);
  static_assert(std::is_same_v<sttl::types::last_t<sttl::typevec<>>, sttl::notype>);

  constexpr auto matcher = sttl::pattern_matching<_1, _2>;
  static_assert(matcher(0, false) == 0);
  static_assert(matcher(0, 0.) == 1);

  static_assert(std::is_same_v<sttl::types::subrange<sttl::typevec<int, float, double, char, bool>, 1, 3>,
                               sttl::typevec<float, double>>);
  static_assert(std::is_same_v<sttl::types::subrange<sttl::typevec<int, float, double, char, bool>, 1>,
                               sttl::typevec<float, double, char, bool>>);
  static_assert(std::is_same_v<sttl::types::subrange<sttl::typevec<int, float, double, char, bool>, 4>,
                               sttl::typevec<bool>>);
  static_assert(std::is_same_v<sttl::types::subrange<sttl::typevec<>, 0>, sttl::typevec<>>);

  static_assert(sttl::find<int, int, float, double> == 0);
  static_assert(sttl::find<int> == sttl::npos);
  static_assert(sttl::find<int, float, double, int> == 2);
  static_assert(sttl::find<int, float, double> == sttl::npos);

  constexpr auto searcher = []<typename T>(std::type_identity<T>) {
    return std::is_same_v<int, T>;
  };
  static_assert(std::is_same_v<sttl::find_if<searcher, int, float, double>, int>);
  static_assert(sttl::is_notype_v<sttl::find_if<searcher>>);
  static_assert(std::is_same_v<sttl::find_if_not<searcher, int, float, double>, float>);
  static_assert(sttl::is_notype_v<sttl::find_if_not<searcher, int, int, int>>);
  static_assert(std::is_same_v<sttl::try_first_t<int, float, double>, int>);
  static_assert(std::is_same_v<sttl::try_first_t<void, void>, void>);
  static_assert(sttl::is_notype_v<sttl::try_first_t<>>);
  static_assert(std::is_same_v<sttl::types::first_t<sttl::typevec<int, float, double>>, int>);
  static_assert(std::is_same_v<sttl::types::first_t<sttl::typevec<void, void>>, void>);
  static_assert(sttl::is_notype_v<sttl::types::first_t<sttl::typevec<>>>);
  static_assert(std::is_same_v<sttl::first_t<int, float, double>, int>);
  static_assert(std::is_same_v<sttl::first_t<void, void>, void>);
  static_assert(std::is_same_v<sttl::last_t<int, float, double>, double>);
  static_assert(std::is_same_v<sttl::last_t<void, void>, void>);
  static_assert(sttl::is_notype_v<sttl::last_t<>>);
  static_assert(std::is_same_v<sttl::types::last_t<sttl::typevec<int, float, double>>, double>);
  static_assert(std::is_same_v<sttl::types::last_t<sttl::typevec<void, void>>, void>);
  static_assert(sttl::is_notype_v<sttl::types::last_t<sttl::typevec<>>>);
}