#pragma once

#include <array>
#include "iq_t.hpp"

#include "_IQNfunctions/_IQNdiv.hpp"
#include "_IQNfunctions/_IQNatan2.hpp"
#include "_IQNfunctions/_IQNtoF.hpp"
#include "_IQNfunctions/_IQFtoN.hpp"
#include "_IQNfunctions/_IQNsqrt.hpp"
#include "_IQNfunctions/_IQNexp.hpp"
#include "_IQNfunctions/_IQNasin_acos.hpp"
#include "_IQNfunctions/_IQNsin_cos.hpp"
#include "_IQNfunctions/_IQNlog.hpp"


namespace iFOC{

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> sinf(const iq_t<P> iq_x){
        return iq_t<Q>(_iq<Q>::from_i32(__iqdetails::__IQNsin_cos(iq_x.value.to_i32(), Q, TYPE_SIN, TYPE_RAD)));
    }

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> cosf(const iq_t<P> iq_x){
        return iq_t<Q>(_iq<Q>::from_i32(__iqdetails::__IQNsin_cos(iq_x.value.to_i32(), Q, TYPE_COS, TYPE_RAD)));
    }

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> sin(const iq_t<P> iq){return sinf<Q>(iq);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> cos(const iq_t<P> iq){return cosf<Q>(iq);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> tanf(const iq_t<P> iq) {return sinf<Q>(iq) / cosf<Q>(iq);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> tan(const iq_t<P> iq) {return tanf<Q>(iq);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> asinf(const iq_t<P> iq) {return iq_t<29>(__iqdetails::_IQNasin(iq.value));}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    requires (Q < 30)
    __fast_inline constexpr iq_t<29> acosf(const iq_t<P> iq) {
        return iq_t<29>(PI/2) - iq_t<29>(__iqdetails::_IQNasin(iq.value));
    }

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    requires (Q < 30)
    __fast_inline constexpr iq_t<Q> asin(const iq_t<P> iq){return asinf(iq);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    requires (Q < 30)
    __fast_inline constexpr iq_t<Q> acos(const iq_t<P> iq){return acosf(iq);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    requires (Q < 30)
    __fast_inline constexpr iq_t<Q> atanf(const iq_t<P> iq) {
        return iq_t<Q>(__iqdetails::_IQNatan2(iq.value, iq_t<P>(1).value));
    }

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    requires (Q < 30)
    __fast_inline constexpr iq_t<Q> atan(const iq_t<P> iq) {
        return atanf(iq);
    }

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    requires (Q < 30)
    __fast_inline constexpr iq_t<Q> atan2f(const iq_t<P> a, const iq_t<P> b) {
        return iq_t<Q>(_iq<Q>(__iqdetails::_IQNatan2<P>(a.value,b.value)));
    }

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    requires (Q < 30)
    __fast_inline constexpr iq_t<Q> atan2(const iq_t<P> a, const iq_t<P> b) {
        return atan2f(a, b);
    }

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> sqrtf(const iq_t<P> iq){
        return iq_t<Q>(__iqdetails::_IQNsqrt(iq.value));
    }

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> sqrt(const iq_t<P> iq){
        return sqrtf(iq);
    }

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> log10(const iq_t<P> iq) {
        return iq_t<Q>(__iqdetails::_IQNlog(iq.value)) / iq_t(__iqdetails::_IQNlog<Q>(iq_t<Q>::from(10).value));
    }

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> log(const iq_t<P> iq) {
        return iq_t<Q>(__iqdetails::_IQNlog(iq.value));
    }

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> exp(const iq_t<P> iq) {
        return iq_t<Q>(__iqdetails::_IQNexp<Q>(iq.value));
    }

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> pow(const iq_t<P> base, const iq_t<P> exponent) {
        return exp(exponent * log(base));
    }

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> pow(const iq_t<P> base, const integral auto times) {
        return exp(times * log(base));
    }


    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> isqrt(const iq_t<P> iq){
        return iq_t<Q>(__iqdetails::_IQNisqrt<P>(iq.value));
    }

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> mag(const iq_t<P> a, const iq_t<P> b){
        return iq_t<Q>(__iqdetails::_IQNmag<P>(a.value, b.value));
    }

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> imag(const iq_t<P> a, const iq_t<P> b){
        return iq_t<Q>(__iqdetails::_IQNimag<P>(a.value, a.value));
    }


}


namespace std{
    using iFOC::iq_t;

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> sinf(const iq_t<P> iq){return iFOC::sinf(iq);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> cosf(const iq_t<P> iq){return iFOC::cosf(iq);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> sin(const iq_t<P> iq){return iFOC::sin(iq);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> cos(const iq_t<P> iq){return iFOC::cos<Q>(iq);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> tanf(const iq_t<P> iq){return iFOC::tanf(iq);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> tan(const iq_t<P> iq){return iFOC::tan(iq);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> asinf(const iq_t<P> iq){return iFOC::asin(iq);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> asin(const iq_t<P> iq){return iFOC::asin(iq);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> acosf(const iq_t<P> iq){return iFOC::acos(iq);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> acos(const iq_t<P> iq){return iFOC::acos(iq);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> atan(const iq_t<P> iq){return iFOC::atan(iq);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> atan2f(const iq_t<P> a, const iq_t<P> b){return iFOC::atan2f(a,b);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> atan2(const iq_t<P> a, const iq_t<P> b){return iFOC::atan2(a,b);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> sqrt(const iq_t<P> iq){return iFOC::sqrt(iq);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> abs(const iq_t<P> iq){return iFOC::abs(iq);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr bool isnormal(const iq_t<P> iq){return iFOC::isnormal(iq);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr bool signbit(const iq_t<P> iq){return iFOC::signbit(iq);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> fmod(const iq_t<P> a, const iq_t<P> b){return iFOC::fmod(a, b);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> mean(const iq_t<P> a, const iq_t<P> b){return iFOC::mean(a, b);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> floor(const iq_t<P> iq){return iFOC::floor(iq);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> ceil(const iq_t<P> iq){return iFOC::ceil(iq);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> log10(const iq_t<P> iq){return iFOC::log10(iq);}

    template<size_t Q = IQ_DEFAULT_Q, size_t P>
    __fast_inline constexpr iq_t<Q> log(const iq_t<P> iq){return iFOC::log(iq);}

}