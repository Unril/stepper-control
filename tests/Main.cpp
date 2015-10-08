#include "stdafx.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

int _tmain(int argc, _TCHAR* argv[]){ 
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
