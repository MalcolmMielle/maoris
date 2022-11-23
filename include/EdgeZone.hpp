#ifndef MAORIS_EDGEZONE_27052016
#define MAORIS_EDGEZONE_27052016

#include <boost/math/distributions/normal.hpp>

#include <Zone.hpp>
#include "bettergraph/SimpleGraph.hpp"

namespace AASS {

namespace maoris {

class EdgeElement;

class EdgeElement {
   protected:
    double _diff;
    bool _breakable;
    bool _fromFuse;

    typedef
        typename bettergraph::SimpleGraph<Zone, EdgeElement>::Vertex VertexZone;

    double min_toward;

   public:
    EdgeElement()
        : _diff(0), _breakable(true), _fromFuse(false), min_toward(-2){};

    EdgeElement(const EdgeElement& ed)
        : _diff(ed.getDiff()), _breakable(true), min_toward(-2){};

    void setMinimum(double in) {
        _fromFuse = true;
        min_toward = in;
    }
    double getMinimum() { return min_toward; }
    double getMinimum() const { return min_toward; }

    bool wasRipple() { return _fromFuse; }
    double getDiff() { return _diff; }
    double getDiff() const { return _diff; }
    void setDiff(double diff) { _diff = diff; }
    void makeUnbreakable() { _breakable = false; }
    bool canRemove() const { return _breakable; }

    bool shouldBeUnbreakable(double max_start,
                             const VertexZone& start,
                             double max_end,
                             const VertexZone& end,
                             double thresh) {
        double max = std::max(max_start, max_end);
        if (min_toward == -2) {
            return false;
        } else if (min_toward <= max - (max * thresh)) {
            return true;
        }
        return false;
    }
};

inline int getDirection(const EdgeElement& edg,
                        const Zone& src,
                        const Zone& targ,
                        double thresh = 0) {
    if (edg.getDiff() < thresh) {
        if (src.getValue() > targ.getValue()) {
            // Going down
            return 1;
        } else {
            // Going Up
            return 2;
        }
    }
    // Same value
    return 0;
}

inline std::ostream& operator<<(std::ostream& in, const EdgeElement& p) {
    in << "d " << p.getDiff();
    return in;
}

inline std::istream& operator>>(std::istream& in, EdgeElement& p) {
    char tmp;
    in >> tmp;
    std::cout << tmp << " ";
    // TODO change to double
    int input;
    in >> input;
    std::cout << input << std::endl;
    p.setDiff(input);
    return in;
}

}  // namespace maoris

}  // namespace AASS

#endif