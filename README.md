# STTL
stl-like algorithms for TYPES

Firstly:
All library works with:
```C++
  -- represents a type pack
  template <typename... Ts>
  struct typevec {
    static constexpr inline size_t size = sizeof...(Ts);
  };
  --
  template <typename T>
  concept pack = instance_of<T, typevec>;
 
  -- means 'no such position'
  constexpr inline size_t npos = static_cast<size_t>(-1);
  -- NOT A TYPE! =) type analogue of npos
  using notype;
```
And some algorithms (such as sort) works with this predicats and projections,
They are functional objects, which accept types in std::type_identity wrapper
```C++
  namespace types {

    template <typename T>
    concept projection = requires(T value) {
      value(std::type_identity<int>{});
    };

    template <typename F>
    concept unary_predicate = projection<F> && requires(F value) {
      { value(std::type_identity<int>{}) } -> std::convertible_to<bool>;
    };

  }
```
Utility:
```C++
namespace sttl {

  template <typename T>
  constexpr inline bool is_notype_v;

  -- works only for templates without NTTPs and template template parameters
  -- true if type an instance of some template
  template <typename T>
  concept specialization = is_specialization<std::remove_cvref_t<T>>;
  
  -- usage example : void foo(instance_of<std::tuple> auto&&... tpls);
  template <typename T, template <typename...> typename OfWhat>
  concept instance_of = specialization<T> && is_instance_of_v<OfWhat, T>;

  template <typename T, typename... Ts>
  constexpr inline size_t count;

  template <typename T, typename... Ts>
  concept one_of = (std::same_as<T, Ts> || ...);

  template<typename T, typename... Ts>
  concept not_a = !one_of<T, Ts...>;

  // compilation error if types are not unique
  // usage example : enum { _ = assert_unique };
  template<typename... Ts>
  constexpr inline bool assert_unique;

  -- for static asserts of smth like
  template<typename...>
  constexpr inline bool always_false = false; 

}  // namespace sttl
```

Algorithms:

```C++

namespace sttl {

  template <types::unary_predicate auto Pred, typename... Ts>
  constexpr inline size_t count_if;

  template <typename T, typename... Ts>
  constexpr inline bool containts;
 
  -- when you need only to store types without any algorithms
  template <typename...>
  struct typelist {};

  -- returns element I from Ts... and sttl::notype if out of range
  template <size_t I, typename... Ts>
  using try_element_t;

  -- returns element I from Ts... or compilation error of out of range
  template <size_t I, typename... Ts>
  using element_t;

  -- returns index of T or npos if more then one such type OR no such type
  template <typename T, typename... Ts>
  constexpr inline size_t find;

  -- returns index of first T in Ts... or npos if no such type
  template <typename T, typename... Ts>
  constexpr inline size_t find_first;

  -- same as std::all_of/none_of/any_of behavior for empty packs
  template <types::unary_predicate auto Pred, typename... Ts>
  constexpr inline bool all_of;

  template <types::unary_predicate auto Pred, typename... Ts>
  constexpr inline bool atleast_one_of;

  template <types::unary_predicate auto Pred, typename... Ts>
  constexpr inline bool none_of;
  
  -- returns pack with types for which Pred returns true
  template <types::unary_predicate auto Pred, typename... Ts>
  using filter;
  
  -- returns index of last T in pack or npos if no such type
  template <typename T, typename... Ts>
  constexpr inline size_t find_last;

  template<std::int_least64_t Value>
  struct integer_literal : constant<Value> {
    // unary operator-
    consteval auto operator-() const;
    // unary operator+
    consteval auto operator+() const;
  
    template <std::integral T>
    requires (std::in_range<T>(Value))
    constexpr operator T() const;
  };

  // NOT NULLTERMINATED
  template <typename Char, size_t N>
  struct fixed_string {
    using char_type;

    std::array<Char, N> chars;

    fixed_string() = default;
    constexpr fixed_string(const Char (&arr)[N + 1]);
    template<typename Traits>
    constexpr fixed_string(std::array<Char, N> arr);
    static constexpr size_t size();
    constexpr auto begin() const;
    constexpr auto end() const;

    constexpr std::basic_string_view<Char> view() const;
  };

  template <typename Char, size_t N0, size_t N1>
  constexpr fixed_string<Char, N0 + N1> operator+(const fixed_string<Char, N0>& left,
                                                  const fixed_string<Char, N1>& right);
                                                 

  template <fixed_string V>
  struct string_literal {
    static constexpr auto value = V;
    using char_type;
    // comparing as NTTP
    consteval auto operator<=>(string_literal) const;
  };

  namespace types {

    template <typename T, pack P>
    constexpr inline size_t count;

    template <unary_predicate auto Pred, pack P>
    constexpr inline size_t count_if;

    template <typename T, pack P>
    constexpr inline bool containts;
   
    -- returns pack with types for which Pred was true 
    template <unary_predicate auto Pred, pack P>
    using filter;

    -- basic block of many algorithms, see std::rotate
    -- rotate<<int, float, double>, 1> --- double int float
    -- rotate <<int, float, double>, -1> --- float double int
    template <pack P, std::int64_t N>
    using rotate;

    -- see std::all_of/none_of/any_of for behavior on empty packs
    template <pack P, unary_predicate auto Pred>
    constexpr inline bool all_of;

    template <pack P, unary_predicate auto Pred>
    constexpr inline bool atleast_one_of;

    template <pack P, unary_predicate auto Pred>
    constexpr inline bool none_of;
   
    -- returns I th element from pack or sttl::notype if out of range
    template <size_t I, pack P>
    using try_element_t;

    -- returns
    -- template <typename T0, typename T1, size_t I>
    -- struct mismatch_result {
    --   using first = T0;                   // type from first pack, where mismatch was. notype if was not
    --   using second = T1;                  // same as first, but from second pack
    --   static constexpr size_t index = I;  // where mismatch was (npos if no mismatch)
    -- };
    template <pack P0, pack P1>
    using mismatch;
    
    -- returns I th element from P or compilation error if out of range
    template <size_t I, pack P>
    using element_t;

    template <unary_predicate auto Pred, pack P>
    using find_if;

    template <unary_predicate auto Pred, pack P>
    using find_if_not;

    template <typename T, pack P>
    constexpr inline size_t find;

    template <typename T, pack P>
    constexpr inline size_t find_first;
    
    -- returns last type from P or sttl::notype if empty pack
    template <pack P>
    using last_t;

    -- returns first type from P or sttl::notype if empty pack
    template <pack P>
    using first_t;

    -- returns new pack with reversed order of types
    template <pack P>
    using reverse;
    
    -- returns pack without first element or sttl::notype if empty pack
    template <pack T>
    using pop_front;
    
    -- returns pack without last element or sttl::notype if empty pack
    template <pack T>
    using pop_back;
    
    -- returns pack with replaced first element to T or sttl::notype if was empty pack
    template <typename T, pack P>
    using replace_front_with;
    
    -- returns pack with replaced last element to T or sttl::notype if was empty pack
    template <typename T, pack P>
    using replace_back_with;

    -- returns new pack with added T before first
    template <typename T, pack U>
    using push_front;

    -- returns new pack with added T after last
    template <typename T, pack U>
    using push_back;
     
    -- returns index of last T in P or npos if no such type
    template <typename T, pack P>
    constexpr inline size_t find_last;

    template <pack Small, pack Big>
    constexpr inline bool starts_with;

    template <pack Small, pack Big>
    constexpr inline bool ends_with;

    -- returns typevec with types [B; E) indexes from pack P
    template <pack P, size_t B, size_t E = P::size>
    using subrange;

    -- returns typevec with template arguments of T or notype if it is not a specialization
    template <typename T>
    using extract;

    -- returns type which is a specialization of Template with types from P
    template <template <typename...> typename Template, pack P>
    using insert;

    -- returns typevec with all template arguments of specializations Ts...
    -- Example: merge<std::vector<int>, typevec<void>> == typevec<int, std::allocator<int>, void>
    template <specialization... Ts>
    using merge;

    -- one of predefined Projectives for sttl::types::sort
    struct type_to_name;
    -- one of predefined Projectives for sttl::types::sort
    struct type_to_size;

    -- projects type with Proj and sorts projected with Compare, returns sorted pack
    template <pack P, projection Proj = type_to_size, typename Compare = std::less<projected_t<Proj>>>
    using sort;

  }  // namespace types

  template <types::unary_predicate auto Pred, typename... Ts>
  using find_if;

  template <types::unary_predicate auto Pred, typename... Ts>
  using find_if_not;

  template <typename... Ts>
  using reverse;

  template <typename... Ts>
  using last_t;

  -- variant Compile Time, stores one type
  -- can contain references, const and volatile types only when initialized from std::type_identity<T>!
  -- can be visited with sttl::visit_ct
  template <typename... Ts>
  struct variant_ct {
    size_t index_ = npos;

    constexpr variant_ct() = default;

    template <typename T>
    requires(containts<std::remove_cvref_t<T>, Ts...>)
    constexpr variant_ct(T&&);

    template <typename T>
    requires(containts<T, Ts...>)
    constexpr variant_ct(std::type_identity<T>);

    constexpr size_t index() const;
  };

  -- visit for variant_ct
  -- visits with exactly one instanciating (std::visit works in O(N!) instantiating...)
  template <instance_of<variant_ct> auto... Vars, typename F>
  constexpr decltype(auto) visit_ct(F&& foo);

  -- is a MATCHER object, accepts arguments and invokes first Foo, which will accept them.
  -- much faster and readable then many if constexpr
  template <auto... Foos>
  constexpr inline auto pattern_matching;
  
  -- returns first element or sttl::npos for empty pack
  template <typename... Ts>
  using try_first_t = typename try_first<Ts...>::type;

  -- returns first from pack or compilation error if empty pack
  -- faster then try_first
  template <typename T0, typename...>
  using first_t = T0;

  -- just C array alias for you
  -- for better quality of fcn code in 2k22
  template<typename T, size_t N>
  using c_array = T[N];

}  // namespace sttl

-- returns sttl::integer_literal and sttl::string_literal, used to create type from value(literal),
-- each value will generate unique type,
-- used for overloading or as NTTP parameters
-- Example : tuple with overloaded for integer_literal operator[]
-- tuple<A, B, C> val { ... };
-- val[1_] // equal to std::get<1>(val);
template <char... Vals>
consteval sttl::integer_literal operator""_i();
template <sttl::fixed_string L>
consteval sttl::string_literal<L> operator""_s();

