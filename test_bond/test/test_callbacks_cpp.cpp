/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <bondcpp/bond.h>
#include <gtest/gtest.h>

#ifndef _WIN32
# include <uuid/uuid.h>
#else
# include <rpc.h>
#endif

#include <ros/spinner.h>

#include <test_bond/TestBond.h>

#include <string>

const char TOPIC[] = "test_bond_topic";
std::string genId()
{
#ifndef _WIN32
  uuid_t uuid;
  uuid_generate_random(uuid);
  char uuid_str[40];
  uuid_unparse(uuid, uuid_str);
  return std::string(uuid_str);
#else
  UUID uuid;
  UuidCreate(&uuid);
  RPC_CSTR str;
  UuidToStringA(&uuid, &str);
  std::string return_string(reinterpret_cast<char *>(str));
  RpcStringFreeA(&str);
  return return_string;
#endif
}

TEST(TestCallbacksCpp, dieInLifeCallback)
{
  std::string id = genId();
  bond::Bond a(TOPIC, id);
  bond::Bond b(TOPIC, id);

  a.setFormedCallback(boost::bind(&bond::Bond::breakBond, &a));

  a.start();
  b.start();

  EXPECT_TRUE(a.waitUntilFormed(ros::Duration(5.0)));
  EXPECT_TRUE(b.waitUntilBroken(ros::Duration(3.0)));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_callbacks_cpp", true);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh;
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  return ret;
};
