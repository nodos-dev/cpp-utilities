/*
 * Copyright MediaZ Teknoloji A.S. All Rights Reserved.
 */
#pragma once

#include <cstdint>
#include <string>
#include <algorithm>
#include <numeric>
#include <queue>
#include <deque>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <set>
#include <vector>
#include <fstream>
#include <variant>
#include <optional>
#include <memory>
#include <array>
#include <filesystem>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <shared_mutex>
#include <atomic>
#include <sstream>
#include <iostream>
#include <chrono>
#include <cassert>
#include <bitset>
#include <ranges>

namespace fs = std::filesystem;


namespace nos::tmp // Utils for template meta-programming
{

template <typename T, typename M> M GetMemberType(M T::*);
template <int a, int... r>
static constexpr int max = max<a, max<r...>>;
template <int a, int b>
static constexpr int max<a, b> = a > b ? a : b;
template <int a>
static constexpr int max<a> = a;

/// Calls a function for each type also passing the index of the type as a constexpr
template<class H, class...T>
struct ForEach
{
	template<typename F>
	static void Do(F&& func)
	{
        Impl(std::move(func));
	}

    template<class F, int i = 0>
    static void Impl(F&& f)
    {
        f.template operator()<H, i>();
        if constexpr (0 != sizeof...(T))
            ForEach<T...>::template Impl<F, i + 1>(std::move(f));
    }
};

template <size_t N>
struct StrLiteral
{
    static constexpr size_t Size = N-1;
	char val[N];
	constexpr StrLiteral(const char (&str)[N]) { std::copy_n(str, N, val); }
	template <typename T>
	constexpr StrLiteral(T str)
	{
		static_assert(!std::is_array<T>::value);
		std::copy_n(str, N, val);
	}
};

template <template <class, class> class Out, class Container1, class Container2>
auto CartesianProduct(Container1 const& a, Container2 const& b)
{
	Out<std::remove_cvref_t<decltype(*a.begin())>, std::remove_cvref_t<decltype(*b.begin())>> re;
	auto x = a.begin();
	auto y = b.begin();
	std::ranges::generate_n(std::inserter(re, re.end()), a.size() * b.size(), [&x, &y, &a, &b] {
		auto re = std::pair{*x, *y};
		if (++y == b.end())
			++x, (y = b.begin());
		return re;
	});
	return re;
}

#define ExplicitStrLiteral(str) nos::tmp::StrLiteral<std::char_traits<char>::length(str) + 1>(str)

template<auto V>
constexpr static auto ConstructName() noexcept {
#ifdef _MSC_VER
	std::string_view sv = __FUNCSIG__;
	auto to = sv.rfind('>')-1;
	for (std::size_t close = sv[to] == ')'; close > 0; ) {
		switch(sv[to = sv.find_last_of(")(", to-1)]) {
		case ')': ++close; break;
		case '(': if (!--close) --to; break;
		}
	}
	for (std::size_t close = sv[to] == '>'; close > 0; ) {
		switch(sv[to = sv.find_last_of("><", to-1)]) {
		case '>': ++close; break;
		case '<': if (!--close) --to; break;
		}
	}
	auto from = sv.find_last_of(">:", to);
	return sv.substr(from + 1, to - from);
#else
	std::string_view sv = __PRETTY_FUNCTION__;
	auto from = sv.rfind(':');
	return sv.substr(from + 1, sv.size() - 2 - from);
#endif
}

template<class From, class Type>
From GetBaseType(Type From::*);

template<class T>
union UnionType {
	char c;
	T f;
	constexpr UnionType() : c{} {}
};

template<class T>
constexpr extern T constexpr_static_init {};

template<auto V>
constexpr static std::string_view GetNameIfExists() noexcept {
	if constexpr (std::is_member_function_pointer_v<decltype(V)>) {
		return ConstructName<V>();
	} else {
#ifdef _MSC_VER
#  if _MSVC_LANG >= 202000
		return ConstructName<&(constexpr_static_init<UnionType<decltype(GetBaseType(V))>>.f.*V)>();
#  else // if_MSVC_LANG < 202000
		return "";
#  endif // _MSVC_LANG >= 202000
#else // if !defined(_MSC_VER)
		return ConstructName<V>();
#endif // _MSC_VER
	}  
}

template<auto N>
constexpr auto GetName(std::string_view name = GetNameIfExists<N>()) {
	return name;
}

}; // namespace nos::tmp

namespace nos
{
   
template<class T, class U>
using pair_vec = std::vector<std::pair<T, U>>;

    
using CommonClock = std::chrono::high_resolution_clock;
using EpochClock = std::chrono::system_clock;

template<template<class> class Out, class Container, class F>
auto Map(Container const& c, F&& pred)
{
    Out<decltype(pred(*c.begin()))> re;
    std::transform(c.begin(), c.end(), std::inserter(re, re.end()), std::move(pred));
    return re;
}


inline void hash_combine(std::size_t& seed) { }

template <typename T, typename... Rest>
inline void hash_combine(std::size_t& seed, const T& v, Rest... rest) {
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
    hash_combine(seed, rest...);
}

template <class T>
using rc = std::shared_ptr<T>;

template <class T, class... Args>
requires(std::is_constructible_v<T, Args...>)
    rc<T> MakeShared(Args&&... args)
{
    return std::make_shared<T>(std::forward<Args>(args)...);
}

template <class T>
using ru = std::unique_ptr<T>;

template <class T, class... Args>
requires(std::is_constructible_v<T, Args...>)
    ru<T> MakeUnique(Args&&... args)
{
    return std::make_unique<T>(std::forward<Args>(args)...);
}

template <class T>
struct Ref : std::reference_wrapper<T>
{
	using std::reference_wrapper<T>::reference_wrapper;
	Ref(T& t) : std::reference_wrapper<T>(t) {}

	T* operator->() const noexcept { return &this->get(); }
	T* operator->() noexcept { return &this->get(); }

	T& operator*() const noexcept { return this->get(); }
	T& operator*() noexcept { return this->get(); }
	
	T* operator&() const noexcept { return &this->get(); }
	T* operator&() noexcept { return &this->get(); }

	Ref* ThisPtr() { return this; }

	bool operator==(const Ref<T>& other) const { return &this->get() == &other.get(); }
};

template <typename T>
struct Nullable
{
	Nullable(T* t) : Ptr(t) {}
	operator bool() const
	{
#ifndef NDEBUG
		Checked = true;
#endif
		return Ptr;
	}

	template <typename U>
	requires std::is_convertible_v<U*, T*>
	Nullable(const Nullable<U>& other) : Ptr(other.GetPtr())
	{
	}
	template <typename U>
	requires std::is_convertible_v<U*, T*>
	Nullable(Nullable<U>&& other) noexcept : Ptr(other.GetPtr())
	{
	}
	template <typename U>
	requires std::is_convertible_v<U*, T*>
	Nullable& operator=(const Nullable<U>& other)
	{
		Ptr = other.GetPtr();
		return *this;
	}
	template <typename U>
	requires std::is_convertible_v<U*, T*>
	Nullable& operator=(Nullable<U>&& other)
	{
		Ptr = other.GetPtr();
		return *this;
	}

	template <typename U>
	requires std::is_convertible_v<U*, T*>
	bool operator==(Nullable<U const> other) const
	{
		return Ptr == other.GetPtr();
	}

	template <typename U>
	requires std::is_convertible_v<U*, T*>
	bool operator==(const U* other) const
	{
		return Ptr == other;
	}

	bool operator==(const T* other) const
	{
		return Ptr == other;
	}

	bool operator==(const T& other) const
	{
		return Ptr == &other;
	}

	T* operator->() 
	{
		assert(Checked);
		return Ptr;
	}

	T& operator*()
	{
		assert(Checked);
		return *Ptr;
	}

	T* GetPtr()
	{
		return Ptr;
	}

protected:
	T* Ptr;
#ifndef NDEBUG
	mutable bool Checked = false;
#endif
};

template<class T, template<class...> class U>
struct SpecializationOf : std::false_type {  };

template<template<class...> class U, class...Args>
struct SpecializationOf<U<Args...>, U> : std::true_type {};

template<class T, template<class...> class U>
concept spec_of = SpecializationOf<std::remove_cvref_t<T>, U>::value;

template<class T>
concept HasEnabledSharedFromThis = requires (T * t)
{
    { t->shared_from_this() } -> spec_of<std::shared_ptr>;
};


template<class T>
concept IsIterator = requires
{
    { (typename std::iterator_traits<T>::iterator_category*)0 };
    { (typename std::iterator_traits<T>::value_type*)0 };
    { (typename std::iterator_traits<T>::difference_type*)0 };
    { (typename std::iterator_traits<T>::pointer*)0 };
};

template<class T>
concept Container = requires (T&& t)
{
    { t.begin() } -> IsIterator;
    { t.end()   } -> IsIterator;
};

template<Container T>
using Contained = typename std::iterator_traits<decltype(std::declval<T>().begin())>::value_type;

template<Container T>
Contained<T>* find(T& v, bool (*F)(Contained<T>&))
{
    if(auto it = std::find_if(v.begin(), v.end(), F); it != v.end())
        return &*it;
    return 0;
}

inline std::string ReadToString(std::string file)
{
    std::ifstream f(file, std::ios::in);
    return std::string((std::istreambuf_iterator<char>(f)),
                       std::istreambuf_iterator<char>());
}

inline std::vector<uint8_t> ReadSpirv(std::string file)
{
	if(!std::filesystem::exists(file))
	{
		return {};
	}
	std::vector<uint8_t> spirv;
	{
		std::ifstream fileStream(file, std::ios::binary);
		fileStream.seekg(0, std::ios::end);
		spirv.resize(fileStream.tellg());
		fileStream.seekg(0, std::ios::beg);
		fileStream.read((char*)spirv.data(), spirv.size());
	}
	return spirv;
}

template <class T>
struct SharedFactory : std::enable_shared_from_this<T>
{
    SharedFactory()                     = default;
    SharedFactory(SharedFactory const&) = delete;

    template <class... Args>
    requires(std::is_constructible_v<T, Args...>) static rc<T> New(Args&&... args)
    {
        return std::make_shared<T>(std::forward<Args>(args)...);
    }
};

template<class T = uint64_t>
struct CircularIndex
{
    T Val;
    T Max;

    explicit CircularIndex(T max) : Val(0), Max((uint64_t)max)
    {
    }
    
    CircularIndex& operator=(T max)
    {
        Val = 0;
		this->max = (uint64_t)max;
        return *this;
    }

    uint64_t operator++()
	{
        return Val = (Val+1) % Max;
    }

    uint64_t operator++(int)
    {
		uint64_t ret = Val % Max;
		Val = (Val + 1) % Max;
        return ret;
    }

    operator uint64_t() const
    {
        return Val % Max;
    }
};

template <typename EnumT, uint32_t MaxEnumVal = (uint32_t)EnumT::Count>
class Flags {
	static_assert(std::is_enum_v<EnumT>, "Flags can only be specialized for enum types");
	using UnderlyingT = typename std::make_unsigned_t<typename std::underlying_type_t<EnumT>>;
public:
	Flags() noexcept = default;
	Flags(EnumT e) { Set(e); }
	Flags(std::initializer_list<EnumT> flags) noexcept { for (auto e : flags) Set(e); }
	Flags& Set(EnumT e, bool value = true) noexcept {
		Bits.set(Underlying(e), value);
		return *this;
	}
	Flags& Reset(EnumT e) noexcept {
		Set(e, false);
		return *this;
	}
	Flags& Reset() noexcept {
		Bits.reset();
		return *this;
	}
	static constexpr Flags All() noexcept { Flags flags; flags.Bits.set(); return flags; }
	Flags operator~() const { Flags ret(*this); ret.Bits = ~ret.Bits; return ret; }
	Flags operator&(Flags const& other) const
	{
		Flags combined;
		combined.Bits = other.Bits & Bits;
		return combined;
	}
	Flags operator|(Flags const& other) const
	{
		Flags combined;
		combined.Bits = other.Bits | Bits;
		return combined;
	}
	bool IsAll() const noexcept { return Bits.all(); }
	bool HasAny() const noexcept { return Bits.any(); }
	bool HasNone() const noexcept { return Bits.none(); }
	constexpr std::size_t Size() const noexcept { return Bits.size(); }
	std::size_t Count() const noexcept { return Bits.count(); }
	constexpr bool operator[](EnumT e) const { return Bits[Underlying(e)]; }
private:
    static constexpr UnderlyingT Underlying(EnumT e) {
        return static_cast<UnderlyingT>(e);
    }
	std::bitset<MaxEnumVal> Bits = {};
};

} // namespace nos

namespace std
{
// Hash for Ref and Nullable
template <class T>
struct hash<::nos::Ref<T>>
{
	std::size_t operator()(const ::nos::Ref<T>& r) const { return std::hash<T*>{}(&r.get()); }
};
template <class T>
struct hash<::nos::Nullable<T>>
{
	std::size_t operator()(const ::nos::Nullable<T>& r) const { return std::hash<T*>{}(r.GetPtr()); }
};
} // namespace std