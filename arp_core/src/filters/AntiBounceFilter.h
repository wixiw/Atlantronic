#pragma once

#include "Eigen/Lgsm"

namespace xde
{
namespace math
{
class AntiBounceFilter
{ 
  public:
    AntiBounceFilter(const double threshold = 0, const size_t tempo = 0)
      : state(false), state_tm1(false), tempo(tempo), cpt(0), threshold(threshold)
    {;}

    bool getState() const { return state; }
    bool transitionDetected() const { return (state_tm1 != state); }

    void setTempo(const size_t tempo_) { tempo = tempo_; reset();}
    void setThreshold(const double threshold_) {threshold = threshold_; reset();}
    void reset() { cpt = tempo;}

    void update(const double rawData)
    {
      state_tm1 = state;
      if(rawData > threshold)
      {
        state = true; reset();
      }
      else
      {
        if(0 == cpt) { state = false;}
        else { cpt--; }
      }
    }

  private:
    bool state;
    bool state_tm1;
    size_t tempo;
    size_t cpt;
    double threshold;
};
}
}