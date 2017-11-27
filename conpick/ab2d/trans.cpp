#include "trans.h"

using namespace ab2d;

Trans::Trans()
    : pos_(Vec2::Zero()), ang_(0)
{
}

Trans::Trans(const Vec2& pos, double ang)
    : pos_(pos), ang_(ang)
{
}

Trans::Trans(double x, double y, double ang)
    : ang_(ang)
{
    pos_[0] = x;
    pos_[1] = y;
}

Trans
Trans::operator*(const Trans& rhs) const
{
    return Trans(pos_ + rotate(rhs.pos_), ang_ + rhs.ang_);
}

Trans&
Trans::operator*=(const Trans& rhs)
{
    pos_ += rotate(rhs.pos_);
    ang_ += rhs.ang_;
    return *this;
}

Trans
Trans::operator*(double scale) const
{
    return Trans(scale * pos_, scale * ang_);
}

Trans&
Trans::operator*=(double scale)
{
    pos_ *= scale;
    ang_ *= scale;
    return *this;
}

Vec2
Trans::rotate(const Vec2& pos) const
{
    Vec2 v;
    const double c = cos(ang_);
    const double s = sin(ang_);
    v[0] = pos[0] * c - pos[1] * s;
    v[1] = pos[0] * s + pos[1] * c;
    return v;
}

Trans
Trans::rotate(const Trans& trans) const
{
    return Trans(rotate(trans.pos_), trans.ang_ + ang_);
}

Trans
Trans::inv() const
{
    return Trans(-pos_, -ang_);
}
