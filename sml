// <!--
// The MIT License (MIT)
//
// Copyright (c) 2024 Kris Jusiak <kris@jusiak.net>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
#if 0
// -->
[Overview](#Overview) / [Examples](#Examples) / [API](#API) / [FAQ](#FAQ)

## SML: UML-2.5 State Machine Language

[![MIT Licence](http://img.shields.io/badge/license-MIT-blue.svg)](https://opensource.org/license/mit)
[![Version](https://img.shields.io/github/v/release/qlibs/sml)](https://github.com/qlibs/sml/releases)
[![Build](https://img.shields.io/badge/build-green.svg)](https://godbolt.org/z/Gcfncoo6r)
[![Try it online](https://img.shields.io/badge/try%20it-online-blue.svg)](https://godbolt.org/z/s9a6EW5j9)

  > https://en.wikipedia.org/wiki/Finite-state_machine

  > https://www.omg.org/spec/UML/2.5.1

### Features

- Single header (https://raw.githubusercontent.com/qlibs/sml/main/sml - for integration see [FAQ](#faq))
- Verifies itself upon include (can be disabled with `-DNTEST` - see [FAQ](#faq))
- Optimized run-time execution, binary size, compilation-times (see [performance](https://godbolt.org/z/W9rP94cYK))
- Minimal [API](#api)

### Requirements

- C++20 ([Clang-15+, GCC-12+](https://en.cppreference.com/w/cpp/compiler_support))

  - No dependencies (no `#include/#import`)
  - No `virtual` used (`-fno-rtti`)
  - No `exceptions` required (`-fno-exceptions`)

---

### Overview

> State Machine (https://godbolt.org/z/s9a6EW5j9)

<p align="center"><img src="https://www.planttext.com/api/plantuml/png/RP313e9034Jl_OfwDK7l7Wo9_WKXPc4RQB8KmXQ-twAoIcHlpRoPQJUFwaQTke1rBqArSY-dGHeuQ4iTuSpLw4H1MGFXBJ40YCMnnFIox8ftZfyKygR_ZcZowfPcCLpMHZmZsHPLuDYQQqDzNHRnTYNsrR5HT-XXoIcGusDsWJsMrZPI9FtpxYoet54_xQARsmprQGR8IRpzA3m1" /></p>

```cpp
// events
struct connect {};
struct established {};
struct ping { bool valid{}; };
struct disconnect {};
struct timeout {};

int main() {
  // guards/actions
  auto establish = [] { std::puts("establish"); };
  auto close     = [] { std::puts("close"); };
  auto reset     = [] { std::puts("reset"); };

  // states
  struct Disconnected {};
  struct Connecting {};
  struct Connected {};

  // transitions
  sml::sm connection = sml::overload{
    [](Disconnected, connect)      -> Connecting   { establish(); },
    [](Connecting,   established)  -> Connected    { },
    [](Connected,    ping event)                   { if (event.valid) { reset(); } },
    [](Connected,    timeout)      -> Connecting   { establish(); },
    [](Connected,    disconnect)   -> Disconnected { close(); },
  };

  static_assert(sizeof(connection) == 1u);

  assert(connection.visit(is<Disconnected>));

  assert(connection.process_event(connect{}));
  assert(connection.visit(is<Connecting>));

  assert(connection.process_event(established{}));
  assert(connection.visit(is<Connected>));

  assert(connection.process_event(ping{.valid = true}));
  assert(connection.visit(is<Connected>));

  assert(connection.process_event(disconnect{}));
  assert(connection.visit(is<Disconnected>));
}
```

```cpp
main: // $CXX -O3 -fno-exceptions -fno-rtti
  push    rax
  lea     rdi, [rip + .L.str.8]
  call    puts@PLT
  lea     rdi, [rip + .L.str.9]
  call    puts@PLT
  lea     rdi, [rip + .L.str.10]
  call    puts@PLT
  xor     eax, eax
  pop     rcx
  ret

.L.str.8:  .asciz  "establish"
.L.str.9:  .asciz  "reset"
.L.str.10: .asciz  "close"
```

---

### Examples

--

### API

```cpp
template<class T>
  requires (requires (T t) { t(); })
struct sm {
  constexpr sm(T&&);
  template<class TEvent, auto dispatch = if_else>
    requires dispatchable<TEvent>
  constexpr auto process_event(const TEvent& event) -> bool ;
  constexpr auto visit(auto&& fn) const;
};
```

```cpp
template<class... Ts> struct overload;
inline constexpr auto if_else; // if_else dispatch policy
inline constexpr auto jmp_table; // jmp_table dispatch policy
struct X {}; // terminate state
```

---

### FAQ

- How to integrate with [CMake.FetchContent](https://cmake.org/cmake/help/latest/module/FetchContent.html)?

    ```
    include(FetchContent)

    FetchContent_Declare(
      qlibs.sml
      GIT_REPOSITORY https://github.com/qlibs/sml
      GIT_TAG v3.0.0
    )

    FetchContent_MakeAvailable(qlibs.sml)
    ```

    ```
    target_link_libraries(${PROJECT_NAME} PUBLIC qlibs.sml);
    ```

- Acknowledgments

  > https://www.youtube.com/watch?v=Zb6xcd2as6o

- Similar projects?
    > [boost.msm](https://github.com/boostorg/msm), [boost.statechart](https://github.com/boostorg/statechart), [boost-ext.sml](https://github.com/boost-ext/sml)
<!--
#endif

#pragma once

namespace sml::inline v3_0_0 {
using size_t = decltype(sizeof(int));
namespace type_traits {
struct none {};
template<class...> inline constexpr bool is_same_v = false;
template<class T> inline constexpr bool is_same_v<T, T> = true;
template<class T> struct remove_reference { using type = T; };
template<class T> struct remove_reference<T&> { using type = T; };
template<class T> struct remove_reference<T&&> { using type = T; };
template<class T> using remove_reference_t = typename remove_reference<T>::type;
template<class T> struct remove_cv { using type = T; };
template<class T> struct remove_cv<const T> { using type = T; };
template<class T> struct remove_cv<volatile T> { using type = T; };
template<class T> struct remove_cv<const volatile T> { using type = T; };
template<class T> using remove_cv_t = typename remove_cv<T>::type;
template<class T> using remove_cvref_t = remove_cv_t<remove_reference_t<T>>;
template<class> struct transition_traits;
template<class T> requires requires { &T::operator(); }
struct transition_traits<T> : transition_traits<decltype(&T::operator())> { };
template<class T> requires requires { &T::template operator()<none>; }
struct transition_traits<T> : transition_traits<decltype(&T::template operator()<none>)> { };
template<class T> requires requires { &T::template operator()<none, none>; }
struct transition_traits<T> : transition_traits<decltype(&T::template operator()<none, none>)> { };
template<class T, class TSrc, class TEvent, class TDst> struct transition_traits<auto (T::*)(TSrc, TEvent) const -> TDst> {
  using src = remove_cvref_t<TSrc>;
  using event = remove_cvref_t<TEvent>;
  using dst = remove_cvref_t<TDst>;
};
template<class T, class TSrc, class TEvent, class TDst> struct transition_traits<auto (T::*)(TSrc, TEvent) -> TDst> {
  using src = remove_cvref_t<TSrc>;
  using event = remove_cvref_t<TEvent>;
  using dst = remove_cvref_t<TDst>;
};
} // namespace type_traits
namespace utility {
template<class T> auto declval() -> T&&;
template<class T, T...> struct integer_sequence { };
template<size_t... Ns> using index_sequence = integer_sequence<size_t, Ns...>;
template<size_t N> using make_index_sequence =
#if defined(__clang__) || defined(_MSC_VER)
  __make_integer_seq<integer_sequence, size_t, N>;
#else
   index_sequence<__integer_pack(N)...>;
#endif
template<class T> struct wrapper {
  [[no_unique_address]] T t;
  constexpr const auto& operator()() const { return t; }
};
} // namespace utility
namespace mp {
template<class...> struct type_list {};
using info = const size_t*;
template<info> struct meta_type { constexpr auto friend get(meta_type); };
template<class T> struct meta_info {
  using value_type = T;
  static constexpr size_t id{};
  constexpr auto friend get(meta_type<&id>) { return meta_info{}; }
};
template<class T> inline constexpr auto meta = &meta_info<T>::id;
template<info meta> using type_of = typename decltype(get(meta_type<meta>{}))::value_type;
template<template<class...> class T, const auto& v>
constexpr auto apply() {
  return []<size_t... Ns>(utility::index_sequence<Ns...>) {
    return T<type_of<v[Ns]>...>{};
  }(utility::make_index_sequence<v.size()>{});
}
template<template<class...> class T, const auto& v>
using apply_t = decltype(apply<T, v>());
} // namespace mp

template<class T, size_t N>
struct static_vector {
  constexpr static_vector() = default;
  constexpr auto push_back(const T& value) { values_[size_++] = value; }
  [[nodiscard]] constexpr const auto& operator[](auto i) const { return values_[i]; }
  [[nodiscard]] constexpr auto begin() const { return &values_[0]; }
  [[nodiscard]] constexpr auto end() const { return &values_[0] + size_; }
  [[nodiscard]] constexpr auto size() const { return size_; }
  [[nodiscard]] constexpr auto empty() const { return not size_; }
  [[nodiscard]] constexpr auto capacity() const { return N; }
  T values_[N]{};
  size_t size_{};
};

template<class... Ts>
  requires (__is_empty(Ts) and ...) and (sizeof...(Ts) < (1u << __CHAR_BIT__))
struct variant {
  template<class T> static constexpr auto id = [] {
    const bool match[]{type_traits::is_same_v<Ts, T>...};
    for (auto i = 0; i < sizeof...(Ts); ++i) if (match[i]) return i;
    return -1;
  }();

  constexpr variant() noexcept = default;
  template<class T> constexpr variant(const T& t) noexcept : index{id<T>} { }

  template<class T> constexpr auto reset(const T&) noexcept requires (type_traits::is_same_v<T, Ts> or ...) {
    index = id<T>;
    return true;
  }

  template<class... TArgs> requires ([]<class T> { return (type_traits::is_same_v<T, TArgs> or ...); }.template operator()<Ts>() or ...)
  constexpr auto reset(const variant<TArgs...>& other) noexcept {
    if (other.index != -1) {
      index = other.index;
      return true;
    }
    return false;
  }

  char index{-1};
};

inline constexpr auto if_else = []<class Fn, template<class...> class T, class... Ts>(Fn&& fn, const T<Ts...>& v) {
  return [&]<size_t... Ns>(utility::index_sequence<Ns...>) {
    return ([&] {
      if (v.index == Ns) return fn(Ts{});
      return false;
    }() or ...);
  }(utility::make_index_sequence<sizeof...(Ts)>{});
};
inline constexpr auto jmp_table = []<class Fn, template<class...> class T, class... Ts>(Fn&& fn, const T<Ts...>& v) {
  static constexpr bool (*dispatch[])(Fn){[](Fn) { return false; }, [](Fn fn) { return fn(Ts{}); }...};
  return dispatch[v.index](fn);
};
template<class... Ts> struct overload : Ts... {
  using value_type = mp::type_list<Ts...>;
  using Ts::operator()...;
};
template<class... Ts> overload(Ts...) -> overload<Ts...>;

template<class T>
  requires (requires (T t) { t(); })
class sm {
  template<class TState>
  static constexpr auto add_state(auto& states, mp::type_list<TState>) {
    const auto new_state = mp::meta<TState>;
    if (new_state == mp::meta<void> or new_state == mp::meta<type_traits::none>) return;
    for (const auto& state : states) if (state == new_state) return;
    states.push_back(new_state);
  }
  template<template<class...> class TList, class... Ts>
  static constexpr auto add_state(auto& states, mp::type_list<TList<Ts...>>) {
    (add_state(states, mp::type_list<Ts>{}), ...);
  }
  static constexpr auto states = []<class... Ts>(mp::type_list<Ts...>) {
    static_vector<mp::info, 2u * sizeof...(Ts)> states{};
    (add_state(states, mp::type_list<typename type_traits::transition_traits<Ts>::src>{}), ...);
    (add_state(states, mp::type_list<typename type_traits::transition_traits<Ts>::dst>{}), ...);
    return states;
  }(typename type_traits::remove_cvref_t<decltype(utility::declval<T>()())>::value_type{});

  template<class TEvent>
  static constexpr auto dispatchable =
    []<class... TStates>(mp::type_list<TStates...>) {
      return (requires(T t, TStates state, TEvent event) { t()(state, event); } or ...);
    }(mp::apply_t<mp::type_list, states>{});

 public:
  constexpr sm(const auto& t) : t_{t} { }

  template<class TEvent, auto dispatch = if_else> requires dispatchable<TEvent>
  constexpr auto process_event(const TEvent& event) -> bool {
    return dispatch([&](const auto& state) {
      if constexpr (requires { states_.reset(t_()(state, event)); }) {
        return states_.reset(t_()(state, event));
      } else if constexpr (requires { t_()(state, event); }) {
        t_()(state, event);
        return true;
      } else {
        return false;
      }
    }, states_);
  }

  template<auto dispatch = if_else> constexpr auto visit(auto&& fn) const {
    return dispatch(fn, states_);
  };

 private:
  [[no_unique_address]] T t_{};
  [[no_unique_address]] mp::apply_t<variant, states> states_ = mp::type_of<states[0u]>{};
};
template<class T> requires requires (T t) { t(); } sm(T) -> sm<T>;
template<class T> sm(T) -> sm<utility::wrapper<T>>;
struct X {}; // terminate state
} // namespace sml

