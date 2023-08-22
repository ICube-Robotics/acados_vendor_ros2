// Copyright 2023 ICUBE Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Thibault Poignonec (tpoignonec@unistra.fr)

#include <gtest/gtest.h>

#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/filesystem.h"

#include "acados_c/ocp_nlp_interface.h"

// Test nlp interface
TEST(test_acados_vendor, test_include_acados_c_template) {
    ocp_nlp_plan_t* plan_ptr = ocp_nlp_plan_create(10);
    EXPECT_NE(plan_ptr, nullptr)
}


