// Copyright 2019 Alexander Liniger

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#include "model_predictive_contouring_control/types.h"
namespace mpcc{

StateVector stateToVector(const State &x)
{
    StateVector xk;
    xk(0) = x.X;
    xk(1) = x.Y;
    xk(2) = x.phi;
    xk(3) = x.s;
    xk(4) = x.v;
    xk(5) = x.delta;
    xk(6) = x.vs;
    return xk;
}

InputVector inputToVector(const Input &u)
{
    InputVector uk = {u.dV,u.dDelta,u.dVs};
    return uk;
}

State vectorToState(const StateVector &xk)
{
    State x;
    x.X     = xk(0);
    x.Y     = xk(1);
    x.phi   = xk(2);
    x.s     = xk(3);
    x.v     = xk(4);
    x.delta = xk(5);
    x.vs    = xk(6);

    return x;
}

Input vectorToInput(const InputVector &uk)
{
    Input u;
    u.dV     = uk(0);
    u.dDelta = uk(1);
    u.dVs    = uk(2);

    return u;
}

State arrayToState(double *xk)
{
    State x;
    x.X     = xk[0];
    x.Y     = xk[1];
    x.phi   = xk[2];
    x.s     = xk[3];
    x.v     = xk[4];
    x.delta = xk[5];
    x.vs    = xk[6];

    return x;
}

Input arrayToInput(double *uk)
{
    Input u;
    u.dV     = uk[0];
    u.dDelta = uk[1];
    u.dVs    = uk[2];

    return u;
}

}