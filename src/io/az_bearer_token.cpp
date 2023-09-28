#include "rosbaz/io/az_bearer_token.h"

namespace rosbaz
{
namespace io
{
BearerToken::BearerToken(const std::string& token) : Azure::Core::Credentials::TokenCredential{"Bearer Token"}, m_token{ token }
{
}

Azure::Core::Credentials::AccessToken
BearerToken::GetToken(Azure::Core::Credentials::TokenRequestContext const& /* tokenRequestContext*/,
                      Azure::Core::Context const& /* context */) const
{
  Azure::Core::Credentials::AccessToken t;
  t.Token = m_token;
  return t;
}

}  // namespace io
}  // namespace rosbaz