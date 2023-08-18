
#include <gtest/gtest.h>

#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/filesystem.h"

#include "acados_c/ocp_nlp_interface.h"

// Test nlp interface
TEST(test_acados_vendor, test_include_acados_c_template) {
    ocp_nlp_plan_t* plan_ptr = ocp_nlp_plan_create(10);
    EXPECT_NE(plan_ptr, nullptr)
}


