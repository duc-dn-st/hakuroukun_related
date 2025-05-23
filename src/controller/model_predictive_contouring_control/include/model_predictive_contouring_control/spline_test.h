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

#ifndef MPCC_SPLINE_TEST_H
#define MPCC_SPLINE_TEST_H


#include "config.h"
#include "cubic_spline.h"
#include "arc_length_spline.h"
namespace mpcc{
int testSpline();
int testArcLengthSpline(const PathToJson &path);
}
#endif //MPCC_SPLINE_TEST_H
