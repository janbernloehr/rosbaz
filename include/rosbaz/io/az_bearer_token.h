#pragma once

#include <azure/core/credentials/credentials.hpp>

#include <string>

namespace rosbaz
{
namespace io
{
class BearerToken : public Azure::Core::Credentials::TokenCredential
{
public:
  explicit BearerToken(const std::string& token);

  Azure::Core::Credentials::AccessToken
  GetToken(Azure::Core::Credentials::TokenRequestContext const& /* tokenRequestContext*/,
           Azure::Core::Context const& /* context */) const override;

private:
  std::string m_token;
};

}  // namespace io
}  // namespace rosbaz
