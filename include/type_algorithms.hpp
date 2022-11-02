
#pragma once

#include <type_traits>
#include <utility>
#include <cstddef>
#include <array>
#include <cstdint>
#include <string_view>
#include <algorithm>
#include <concepts>
#include <optional>
#include <bit>
#include <cassert>
#include <functional>
#include <limits>

// TODO waiting for support
#ifdef _MSC_VER
#ifndef __clang__
#define CONSTEVAL consteval
#else
#define CONSTEVAL constexpr
#endif
#else
#define CONSTEVAL constexpr
#endif
// Yes, msvc do not support EBO which is already GUARANTEED by C++ standard for ~13 years
#if defined(_MSC_VER)
#define AA_MSVC_EBO __declspec(empty_bases)
#else
#define AA_MSVC_EBO
#endif
namespace sttl {

  template <typename, size_t>
  struct typenode {};

}  // namespace sttl

namespace noexport {
  template <typename, typename...>
  struct AA_MSVC_EBO typevecimpl;

  template <size_t... Is, typename... Ts>
  struct AA_MSVC_EBO typevecimpl<std::index_sequence<Is...>, Ts...> : ::sttl::typenode<Ts, Is>... {};

  // for integer_literal
  CONSTEVAL std::int_least64_t parse_int(std::initializer_list<char> list) {
    std::int_least64_t result = 0;
    for (auto v : list) {
      std::int_least64_t digit = v - '0';
      if (digit > 9 || digit < 0)
        throw false;
      result += digit;
      result *= 10;
    }
    result /= 10;
    return result;
  }

  template <typename T>
  struct debug {
    debug() = delete;
  };
}  // namespace noexport

namespace sttl {

  // UTILITY

  // prints type in compilation error
  template <typename T>
  constexpr inline auto debug_print = noexport::debug<T>{};

  // CONCEPT ONE_OF

  template <typename T, typename... Ts>
  concept one_of = (std::same_as<T, Ts> || ...);

  // NULL_T like void but better(may be in tuple for example or returned from function by value)
  struct null_t {
    // for using as NTTP
    constexpr bool operator==(null_t) const noexcept {
      return true;
    }
    template<one_of<std::nullptr_t, std::nullopt_t> T>
    CONSTEVAL operator T() const noexcept {
      struct _ {
      } any_empty;
      return std::bit_cast<T>(any_empty);
    }
  };

  constexpr inline null_t null = {};

  template <typename T>
  constexpr inline bool is_null_v = std::is_same_v<null_t, std::remove_cvref_t<T>>;

  template <typename... Ts>
  struct typevec : noexport::typevecimpl<std::index_sequence_for<Ts...>, Ts...> {
    static constexpr inline size_t size = sizeof...(Ts);
  };

  template <>
  struct typevec<> {
    static constexpr inline size_t size = 0;

    constexpr bool operator==(typevec) const noexcept {
      return true;
    }
  };

  // NOT A TYPE! =) type analogue of npos
  using notype = union {};

  template <typename T>
  constexpr inline bool is_notype_v = std::is_same_v<notype, std::remove_cvref_t<T>>;

  constexpr inline size_t npos = static_cast<size_t>(-1);

  template <auto V>
  struct constant {
    using type = decltype(V);
    static constexpr auto value = V;
  };

  template <typename T0, typename T1, size_t I>
  struct mismatch_result {
    using first = T0;                   // type from first pack, where mismatch was. notype if was not
    using second = T1;                  // same as first, but from second pack
    static constexpr size_t index = I;  // where mismatch was (npos if no mismatch)
  };

  // is_instance_of
  template <template <typename...> typename Template, typename TypeToCheck>
  struct is_instance_of {
   private:
    template <typename>
    struct check : std::false_type {};
    template <typename... Args>
    struct check<Template<Args...>> : std::true_type {};

   public:
    static constexpr inline bool value = check<std::remove_cvref_t<TypeToCheck>>::value;
  };

  // CONCEPT SPECIALIZATION - works only for templates without NTTPs and template template parameters

  template <typename>
  constexpr inline bool is_specialization = false;

  template <typename... Ts, template <typename...> typename T>
  constexpr inline bool is_specialization<T<Ts...>> = true;

  template <typename T>
  concept specialization = is_specialization<std::remove_cvref_t<T>>;

  template <template <typename...> typename Template, typename TypeToCheck>
  constexpr inline bool is_instance_of_v = is_instance_of<Template, TypeToCheck>::value;

  template <typename T, template <typename...> typename OfWhat>
  concept instance_of = specialization<T> && is_instance_of_v<OfWhat, T>;

  template <typename T, typename... Ts>  // 0 for case when sizeof...(Ts) == 0
  constexpr inline size_t count = (0 + ... + std::is_same_v<T, Ts>);

  // CONCEPT NOT_A
  
  template<typename T, typename... Ts>
  concept not_a = !one_of<T, Ts...>;

  // UTILITY ASSERT UNIQUE

  template <typename... Ts>
  struct assert_unique_impl : std::type_identity<Ts>... {};

  // compilation error if types are not unique
  template<typename... Ts>
  constexpr inline bool assert_unique = (assert_unique_impl<Ts...>{}, true);

  // UTILITY ALWAYS_FALSE

  template<typename...>
  constexpr inline bool always_false = false; 

  namespace types {

    template <typename T>
    concept projection = requires(T value) {
      value(std::type_identity<int>{});
    };
    template <projection Proj>
    using projected_t = decltype(std::remove_cvref_t<Proj>{}(std::type_identity<int>{}));

    template <typename T, projection Proj>
    constexpr inline auto projected_v = Proj{}(std::type_identity<T>{});

    template <typename F>
    concept unary_predicate = projection<F> && std::convertible_to<bool, projected_t<F>>;

  }  // namespace types

  template <types::unary_predicate auto Pred, typename... Ts>
  constexpr inline size_t count_if = (0 + ... + static_cast<bool>(Pred(std::type_identity<Ts>{})));

  template <typename T, typename... Ts>
  constexpr inline bool containts = count<T, Ts...> != 0;

}  // namespace sttl

namespace noexport {

  template <size_t I, typename T>
  CONSTEVAL auto try_typevec_element(::sttl::typenode<T, I>) -> std::type_identity<T>;

  template <size_t I, typename T>
  CONSTEVAL auto try_typevec_element_impl(int) -> decltype(try_typevec_element<I>(T{}));
  template <size_t, typename>
  CONSTEVAL auto try_typevec_element_impl(...) -> std::type_identity<sttl::notype>;

  template <size_t I, typename T>
  CONSTEVAL auto typevec_element(::sttl::typenode<T, I>&&) -> std::type_identity<T>;

  template <typename T, size_t I>
  CONSTEVAL size_t typevec_index(::sttl::typenode<T, I>&&) noexcept {
    return I;
  }
  template <typename T, typename Pack>
  CONSTEVAL auto typevec_index_impl(int)->decltype(typevec_index<T>(Pack{})) {
    return typevec_index<T>(Pack{});
  }
  template <typename, typename>
  CONSTEVAL size_t typevec_index_impl(...) {
    return ::sttl::npos;
  }

  // string_view but aggregate(no private memberes) and can only compare for using on compile time
  struct string_view {
    const char* ptr = nullptr;
    size_t size = 0;

    //  workaround for bug with <=> on compilation (dont see it)
    constexpr bool operator==(const string_view& other) const noexcept {
      return std::string_view{ptr, size} == std::string_view{other.ptr, other.size};
    }
    constexpr auto operator<=>(const string_view& other) const noexcept {
      return std::string_view{ptr, size} <=> std::string_view{other.ptr, other.size};
    }
  };

}  // namespace noexport

// computations in initializer list ordered by C++ standard
#define ORDERED (void)std::initializer_list<bool>

namespace sttl {

  // when you need only to store types without any algorithms
  template <typename...>
  struct typelist {};

  template <typename T>
  concept pack = instance_of<T, typevec>;

  // returns element I from Ts... and sttl::notype if I >= sizeof...(Ts)
  template <size_t I, typename... Ts>
  using try_element_t = typename decltype(noexport::try_typevec_element_impl<I, typevec<Ts...>>(0))::type;

  // returns element I from Ts...
  template <size_t I, typename... Ts>
  using element_t = typename decltype(noexport::typevec_element<I>(typevec<Ts...>{}))::type;

  // returns index of T or sttl::npos if count of T in Ts != 1
  template <typename T, typename... Ts>
  constexpr inline size_t find = noexport::typevec_index_impl<T, typevec<Ts...>>(0);

  // returns index of first T in Ts...
  template <typename T, typename... Ts>
  constexpr inline size_t find_first = [] {
    size_t result = sttl::npos;
    size_t i = -1;
    // for guaranteed sequence
    ORDERED{((++i, std::is_same_v<T, Ts>) && result == sttl::npos && (result = i))...};
    return result;
  }  // INVOKED HERE
  ();

  template <auto Pred, typename... Ts, size_t... Is>
  CONSTEVAL size_t find_if_impl(typevec<Ts...>, std::index_sequence<Is...>) {
    size_t index = npos;
    ORDERED{(static_cast<bool>(Pred(std::type_identity<Ts>{})) && index == npos && (index = Is))...};
    return index;
  }
  template <auto Pred, typename... Ts, size_t... Is>
  CONSTEVAL size_t find_if_not_impl(typevec<Ts...>, std::index_sequence<Is...>) {
    size_t index = npos;
    ORDERED{(!static_cast<bool>(Pred(std::type_identity<Ts>{})) && index == npos && (index = Is))...};
    return index;
  }

  template <types::unary_predicate auto Pred, typename... Ts>
  constexpr inline bool all_of = (static_cast<bool>(Pred(std::type_identity<Ts>{})) && ...);

  template <types::unary_predicate auto Pred, typename... Ts>
  constexpr inline bool atleast_one_of = (static_cast<bool>(Pred(std::type_identity<Ts>{})) || ...);

  template <types::unary_predicate auto Pred, typename... Ts>
  constexpr inline bool none_of = sizeof...(Ts) == 0
                                      ? true
                                      : (!static_cast<bool>(Pred(std::type_identity<Ts>{})) || ...);

  // N may be < 0, but even in this case its correctly works
  template <size_t N, typename... Ts, size_t... Is>
  CONSTEVAL auto rotate_impl(typevec<Ts...>, std::index_sequence<Is...>)
      -> typevec<element_t<(N + Is) % sizeof...(Is), Ts...>...>;

  template <auto Pred, typename... Ts>
  CONSTEVAL auto filter_impl(typevec<Ts...> = {}) {
    constexpr auto indexes = [] {
      std::array<size_t, count_if<Pred, Ts...>> indexes{};
      size_t array_index = 0;
      size_t type_index = 0;
      ORDERED{(++type_index, static_cast<bool>(Pred(std::type_identity<Ts>{})) &&
                                 (indexes[array_index++] = type_index - 1))...};
      return indexes;
    }();  // INVOKED HERE
    // Array, TsCopy etc its all because of FUCKING MSVC BUGS
    return [&]<size_t... Is, typename... TsCopy, auto Arr>(std::index_sequence<Is...>, typelist<TsCopy...>,
                                                           constant<Arr>) {
      return typevec<element_t<Arr[Is], TsCopy...>...>{};
    }  // INVOKED HERE
    (std::make_index_sequence<count_if<Pred, Ts...>>{}, typelist<Ts...>{}, constant<indexes>{});
  }

  template <types::unary_predicate auto Pred, typename... Ts>
  using filter = decltype(filter_impl<Pred, Ts...>());  // INVOKED HERE

  template <typename T, typename... Ts>
  constexpr inline size_t find_last = [] {
    size_t index = npos;
    size_t i = -1;
    ORDERED{((++i, std::is_same_v<T, Ts>) && (index = i))...};
    return index;
  }  // INVOKED HERE
  ();

  // Why int_least64_t?
  // - no unsigned type because may be signed
  // - no intmax_t, because can have different MAX on different platforms,
  // so possible a compilation error for same code like integer_literal<1000000>; 
  //- int64_t is optional by C++ standard, platform can not support it
  template<std::int_least64_t Value>
  struct integer_literal : constant<Value> {
    // must be <= int64_t max, for case when int_least64 is more then 64 bit, same compilations on every platform
    static_assert(Value <= 0xff'ff'ff'ff'ff'ff'ff'ff);
    CONSTEVAL auto operator-() const noexcept {
      return integer_literal<-Value>{};
    }
    CONSTEVAL auto operator+() const noexcept {
      return integer_literal<Value>{};
    }
    template <std::integral T>
    requires (std::in_range<T>(Value))
    constexpr operator T() const noexcept {
      return Value;
    }
  };

  #if defined(__clang__) || defined(__GNUC__)
  #define PRETTY_FUNC_NAME __PRETTY_FUNCTION__
  #elif defined(_MSC_VER)
  #define PRETTY_FUNC_NAME __FUNCSIG__
  #endif
  // NOT NULLTERMINATED
  template <typename Char, size_t N>
  struct fixed_string {
    using char_type = Char;

    std::array<Char, N> chars;

    fixed_string() = default;

    constexpr fixed_string(const Char (&arr)[N + 1]) noexcept {
      std::copy_n(std::begin(arr), N, chars.begin());
    }
    template<typename Traits>
    constexpr fixed_string(std::array<Char, N> arr) noexcept : chars(arr) {
    }
    static constexpr size_t size() noexcept {
      return N;
    }
    constexpr auto begin() noexcept {
      return chars.begin();
    }
    constexpr auto begin() const noexcept {
      return chars.begin();
    }
    constexpr auto end() noexcept {
      return chars.end();
    }
    constexpr auto end() const noexcept {
      return chars.end();
    }

    constexpr std::basic_string_view<Char> view() const noexcept {
      return {chars.data(), chars.size()};
    }
  };

  template <typename Char, size_t N0, size_t N1>
  constexpr fixed_string<Char, N0 + N1> operator+(const fixed_string<Char, N0>& left,
                                                         const fixed_string<Char, N1>& right) noexcept {
    fixed_string<Char, N0 + N1> result{};
    auto it = std::copy(left.begin(), left.end(), result.begin());
    std::copy(right.begin(), right.end(), it);
    return result;
  }

  template <typename Char, size_t N>
  fixed_string(Char (&)[N]) -> fixed_string<std::remove_const_t<Char>, N - 1>;

  template <typename T>
  CONSTEVAL auto type_name_impl() {
    return fixed_string<char, sizeof(PRETTY_FUNC_NAME) - 1>{PRETTY_FUNC_NAME};
  }
  template <typename T>
  constexpr inline auto type_name = type_name_impl<T>();

  template <fixed_string V>
  struct string_literal {
    static constexpr auto value = V;

    using char_type = typename decltype(V)::char_type;

    // comparing as NTTP

    CONSTEVAL bool operator==(string_literal) const noexcept {
      return true;
    }
    template <fixed_string L>
    CONSTEVAL bool operator==(string_literal<L>) const noexcept {
      return false;
    }
    CONSTEVAL auto operator<=>(string_literal) const noexcept {
      return std::strong_ordering::equal;
    }
    template <fixed_string L>
    CONSTEVAL auto operator<=>(string_literal<L>) const noexcept {
      return V <=> L;
    }

  };

  namespace types {

    template <typename T, pack P>
    constexpr inline size_t count = []<typename... Ts>(typevec<Ts...>) {
      return (0 + ... + std::is_same_v<T, Ts>);
    }  // INVOKED HERE
    (P{});

    template <unary_predicate auto Pred, pack P>
    constexpr inline size_t count_if = []<typename... Ts>(typevec<Ts...>) {
      return ::sttl::count_if<Pred, Ts...>;
    }  // INVOKED HERE
    (P{});

    template <typename T, pack P>
    constexpr inline bool containts = count<T, P> != 0;

    template <unary_predicate auto Pred, pack P>
    using filter = decltype(filter_impl<Pred>(P{}));

    // rotate<<int, float, double>, 1> --- double int float
    // rotate <<int, float, double>, -1> --- float double int
    template <pack P, std::int64_t N>
    using rotate =
        decltype(rotate_impl < N >= 0 ? N : P::size + N > (P{}, std::make_index_sequence<P::size>{}));

    template <pack P, unary_predicate auto Pred>
    constexpr inline bool all_of = []<typename... Ts>(typevec<Ts...>) {
      return ::sttl::all_of<Pred, Ts...>;
    }  // INVOKED HERE
    (P{});

    template <pack P, unary_predicate auto Pred>
    constexpr inline bool atleast_one_of = []<typename... Ts>(typevec<Ts...>) {
      return ::sttl::atleast_one_of<Pred, Ts...>;
    }  // INVOKED HERE
    (P{});

    template <pack P, unary_predicate auto Pred>
    constexpr inline bool none_of = []<typename... Ts>(typevec<Ts...>) {
      return ::sttl::none_of<Pred, Ts...>;
    }  // INVOKED HERE
    (P{});

    template <size_t I, pack P>
    using try_element_t = typename decltype(noexport::try_typevec_element_impl<I, P>(0))::type;

    template <pack P0, pack P1, size_t... Is>
    CONSTEVAL auto mismatch_impl(P0, P1, std::index_sequence<Is...>) {
      constexpr size_t index = [] {
        size_t index = npos;
        ((!std::is_same_v<try_element_t<Is, P0>, try_element_t<Is, P1>> && index == npos && (index = Is)),
         ...);
        return index;
      }();  // INVOKED HERE
      return mismatch_result<try_element_t<index, P0>, try_element_t<index, P1>, index>{};
    }

    template <pack P0, pack P1>
    using mismatch =
        decltype(mismatch_impl(P0{}, P1{}, std::make_index_sequence<std::min(P0::size, P1::size)>{}));

    template <size_t I, pack P>
    using element_t = typename decltype(noexport::typevec_element<I>(P{}))::type;

    template <unary_predicate auto Pred, pack P>
    using find_if = try_element_t<find_if_impl<Pred>(P{}, std::make_index_sequence<P::size>{}), P>;

    template <unary_predicate auto Pred, pack P>
    using find_if_not = try_element_t<find_if_not_impl<Pred>(P{}, std::make_index_sequence<P::size>{}), P>;

    template <typename T, pack P>
    constexpr inline size_t find = noexport::typevec_index_impl<T, P>(0);

    template <typename T, pack P>
    constexpr inline size_t find_first = []<typename... Ts>(typevec<Ts...>) {
      return ::sttl::find_first<T, Ts...>;
    }  // INVOKED HERE
    (P{});

    template <pack P>
    using last_t = try_element_t<P::size - 1, P>;

    template <pack P>
    using first_t = try_element_t<0, P>;

    template <typename... Ts, size_t... Is>
    CONSTEVAL auto reverse_impl(typevec<Ts...>, std::index_sequence<Is...>)
        -> typevec<element_t<sizeof...(Is) - 1 - Is, typevec<Ts...>>...>;

    template <pack P>
    using reverse = decltype(reverse_impl(P{}, std::make_index_sequence<P::size>{}));

    template <typename T0, typename... Ts>
    CONSTEVAL auto pop_front_impl(typevec<T0, Ts...>) -> typevec<Ts...>;
    CONSTEVAL auto pop_front_impl(typevec<>) -> notype;

    template <pack T>
    using pop_front = decltype(pop_front_impl(T{}));

    template <pack P>
    using pop_back = typename decltype([] {
      if constexpr (P::size == 0)
        return std::type_identity<notype>{};
      else
        return std::type_identity<reverse<pop_front<reverse<P>>>>{};
    }())::type;

    template <typename What, typename T0, typename... Ts>
    CONSTEVAL auto replace_front_with_impl(typevec<T0, Ts...>) -> typevec<What, Ts...>;
    template <typename>
    CONSTEVAL auto replace_front_with_impl(typevec<>) -> notype;

    template <typename T, pack P>
    using replace_front_with = decltype(replace_front_with_impl<T>(P{}));

    template <typename T, pack P>
    using replace_back_with = reverse<replace_front_with<T, reverse<P>>>;

    template <typename T, typename... Ts>
    CONSTEVAL auto push_front_impl(typevec<Ts...>) -> typevec<T, Ts...>;

    template <typename T, typename... Ts>
    CONSTEVAL auto push_back_impl(typevec<Ts...>) -> typevec<Ts..., T>;

    template <typename T, pack U>
    using push_front = decltype(push_front_impl<T>(U{}));

    template <typename T, pack U>
    using push_back = decltype(push_back_impl<T>(U{}));

    template <typename T, pack P>
    constexpr inline size_t find_last = []<typename... Ts>(typevec<Ts...>) {
      return ::sttl::find_last<T, Ts...>;
    }  // INVOKED HERE
    (P{});

    template <pack Small, pack Big>
    constexpr inline bool starts_with = mismatch<Small, Big>::index == npos;

    template <pack Small, pack Big>
    constexpr inline bool ends_with = starts_with<Small, reverse<Big>>;

    template <typename Pack, size_t B, size_t E, size_t... Is>
    CONSTEVAL auto subrange_impl(std::index_sequence<Is...>) -> typevec<element_t<Is + B, Pack>...>{};

    // returns typevec with types [B; E) indexes from pack P
    template <pack P, size_t B, size_t E = P::size>
    using subrange = decltype(subrange_impl<P, B, E>(std::make_index_sequence<E - B>{}));

    template <template <typename...> typename Template, typename... Ts>
    CONSTEVAL auto extract_impl(Template<Ts...>) -> typevec<Ts...>;
    CONSTEVAL auto extract_impl(auto&&) -> notype;

    // returns typevec with template arguments of T or notype if it is not a specialization
    template <typename T>
    using extract = decltype(extract_impl(std::declval<T>()));

    template <template <typename...> typename Template, typename... Ts>
    CONSTEVAL auto insert_impl(typevec<Ts...>) -> Template<Ts...>;

    // returns type which is a specialization of Template with types from P
    template <template <typename...> typename Template, pack P>
    using insert = decltype(insert_impl<Template>(P{}));

    template <typename P, typename...>
    struct merge_impl : std::conditional<pack<P>, P, notype> {};

    template <template <typename...> typename T, template <typename...> typename U, typename... A,
              typename... B, typename... Tail>
    struct merge_impl<T<A...>, U<B...>, Tail...> : merge_impl<typevec<A..., B...>, Tail...> {};

    // returns typevec with all template arguments of specializations Ts...
    // Example: merge<std::vector<int>, typevec<void>> == typevec<int, std::allocator<int>, void>
    template <specialization... Ts>
    using merge = typename merge_impl<Ts...>::type;

    struct type_to_name {
      template <typename T>
      constexpr noexport::string_view operator()(std::type_identity<T>) noexcept {
        return noexport::string_view{type_name<T>.chars.data(), type_name<T>.chars.size()};
      }
    };
    struct type_to_size {
      template <typename T>
      constexpr size_t operator()(std::type_identity<T>) noexcept {
        return sizeof(T);
      }
    };
    template <typename Proj, projected_t<Proj> V>
    struct sort_pred_impl {
      static constexpr projected_t<Proj> value = V;

      template <typename T>
      constexpr bool operator()(std::type_identity<T>) const noexcept {
        return value == projected_v<T, Proj>;
      }
    };

    template <projection Proj, typename... Types, typename Compare>
    CONSTEVAL auto sort_impl(typevec<Types...>, Compare compare = {}, Proj proj = {}) {
      if constexpr (sizeof...(Types) < 2)
        return typevec<Types...>{};
      else {
        constexpr size_t sz = [compare] {
          std::array vec{projected_v<Types, Proj>...};
          std::sort(vec.begin(), vec.end(), compare);
          auto last = std::unique(vec.begin(), vec.end()); // TODO compare here same as for sort?
          return std::distance(vec.begin(), last);
        }();
        constexpr auto arr = [compare]<size_t Sz>(constant<Sz>) {
          std::array vec{projected_v<Types, Proj>...};
          std::sort(vec.begin(), vec.end(), compare);
          (void)std::unique(vec.begin(), vec.end()); // TODO compare here same as for sort ?
          std::array<projected_t<Proj>, Sz> result;
          std::copy_n(vec.begin(), Sz, result.begin());
          return result;
        }(constant<sz>{});

        return [&]<size_t... Is>(std::index_sequence<Is...>) {
          return merge<filter<sort_pred_impl<Proj, arr[Is]>{}, typevec<Types...>>...>{};
        }
        (std::make_index_sequence<sz>{});
      }
    }
    // TODO - change default to type_to_name when clang will have possibility to MANGLE it
    template <pack P, projection Proj = type_to_size, typename Compare = std::less<projected_t<Proj>>>
    using sort = decltype(sort_impl(P{}, Compare{}, Proj{}));

  }  // namespace types

  template <types::unary_predicate auto Pred, typename... Ts>
  using find_if = types::find_if<Pred, typevec<Ts...>>;

  template <types::unary_predicate auto Pred, typename... Ts>
  using find_if_not = types::find_if_not<Pred, typevec<Ts...>>;

  template <typename... Ts>
  using reverse = types::reverse<typevec<Ts...>>;

  template <typename... Ts>
  using last_t = typename decltype((std::type_identity<notype>{}, ..., std::type_identity<Ts>{}))::type;

  // variant Compile Time
  // can contain references, const and volatile types only when initialized from std::type_identity<T>!
  template <typename... Ts>
  struct tagged_enum {
    using size_type = std::conditional_t<(sizeof...(Ts) < std::numeric_limits<uint_least8_t>::max()),
                                         std::uint_least8_t, std::size_t>;
    // not private because it must be literal type
    size_type _index = static_cast<size_type>(-1);

    constexpr tagged_enum() = default;

    // precondition : 'i' must be in range [0, sizeof...(Ts))
    constexpr tagged_enum(size_type i) noexcept : _index(i) {
      assert(i < sizeof...(Ts));
    }
    // precondition : 'i' must be in range [0, sizeof...(Ts))
    constexpr tagged_enum& operator=(size_type i) noexcept {
      assert(i < sizeof...(Ts));
      _index = i;
      return *this;
    }
    // clang-format off
    template <typename T>
    requires(containts<T, Ts...>)
    constexpr tagged_enum(std::type_identity<T>) noexcept
        : _index(find_first<T, Ts...>)
    {}
    // clang-format on
    constexpr tagged_enum(const tagged_enum&) = default;
    constexpr tagged_enum(tagged_enum&&) = default;
    constexpr tagged_enum& operator=(const tagged_enum&) noexcept = default;
    constexpr tagged_enum& operator=(tagged_enum&&) noexcept = default;

    constexpr size_t index() const noexcept {
      return _index;
    }
  };

  // visit for tagged_enum

  // exactly one instanciation of function F
  template <instance_of<tagged_enum> auto... Vars, typename F>
  constexpr decltype(auto) visit_enum(F&& foo) {
    return std::forward<F>(foo)(
        std::type_identity<types::element_t<Vars.index(), types::extract<decltype(Vars)>>>{}...);
  }
  // similar to std::visit, but for enum
  template <typename F, typename... Ts>
  constexpr decltype(auto) visit_enum(F&& f, tagged_enum<Ts...> ts) {
    constexpr std::array tbl{[]<typename T>(std::type_identity<T>) {
      return +[](F&& _f) {
        return _f(std::type_identity<Ts>{});
      };
    }(std::type_identity<Ts>{})...};
    return tbl[ts.index()](std::forward<F>(f));
  }
  template<typename F, typename V1, typename... Vs>
  constexpr decltype(auto) visit_enum(F&& f, V1 var, Vs... vars) {
    return ::sttl::visit_enum(
        [&](auto&& v) { return ::sttl::visit_enum(std::bind_front(std::forward<F>(f), v), vars...); }, var);
  }

  // Foos must be functions(possibly template), atleast one of them must accept all input arguments.
  // Foos also must be unique types
  template <typename... Foos>
  struct pattern_matching_fn {
    enum { check = assert_unique<Foos...> };

    template <typename... Ts>
    constexpr decltype(auto) operator()(Ts&&... args) const {
      constexpr auto searcher = []<typename T>(std::type_identity<T>) {
        return std::is_invocable_v<T, Ts&&...>;
      };
      using what_invoke = find_if<searcher, Foos...>;
      static_assert(!is_notype_v<what_invoke>, "NOT MATCHED");
      return what_invoke{}(std::forward<Ts>(args)...);
    }
  };

  template <auto... Foos>
  constexpr inline auto pattern_matching = pattern_matching_fn<decltype(Foos)...>{};

  template <typename...>
  struct try_first : std::type_identity<notype> {};

  template <typename T0, typename... Ts>
  struct try_first<T0, Ts...> : std::type_identity<T0> {};

  template <typename... Ts>
  using try_first_t = typename try_first<Ts...>::type;

  template <typename T0, typename...>
  using first_t = T0;

  // for better quality of fcn code in 2k22
  template<typename T, size_t N>
  using c_array = T[N];

  // for creating concepts for checking if invocable without conversations
  // Example:
  // template<typename T> concept X = requires(exactly<int> i, T value) { T.foo(i); }
  // checks if T has foo which accepts exactly int without conversations (or accepts any type)
  template<typename T>
  struct exactly {
    template <std::same_as<T> U>
    constexpr operator U() noexcept;
  };

}  // namespace sttl

// in global namespace because its really global(for all literals),
// and dont need using namespace...
template <sttl::fixed_string L>
CONSTEVAL auto operator""_fixs() noexcept {
  return L;
}

// int literal
template <char... Vals>
CONSTEVAL auto operator""_i() noexcept {
  constexpr std::array arr{Vals...};
  static_assert(!(sizeof...(Vals) > 1 && arr[0] == '0'), "Octal integer literals are not supported");
  return sttl::integer_literal<noexport::parse_int({Vals...})>{};
}
// TODO rename _s into _ and _i into _ for common interface(now bug in MSVC)
// string literal
template <sttl::fixed_string L>
CONSTEVAL auto operator""_s() noexcept {
  return sttl::string_literal<L>{};
}

