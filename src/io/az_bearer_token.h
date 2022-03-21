#pragma once

#include <azure/core/credentials/credentials.hpp>

#include <string>

class BearerToken : public Azure::Core::Credentials::TokenCredential
{
public:
  explicit BearerToken(const std::string& token) : m_token{ token }
  {
  }

  Azure::Core::Credentials::AccessToken
  GetToken(Azure::Core::Credentials::TokenRequestContext const& /* tokenRequestContext*/,
           Azure::Core::Context const& /* context */) const override
  {
    Azure::Core::Credentials::AccessToken t;
    t.Token = m_token;
    return t;
  }

private:
  std::string m_token;
};