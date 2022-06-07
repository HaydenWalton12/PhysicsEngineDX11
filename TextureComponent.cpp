#include "TextureComponent.h"


TextureComponent::TextureComponent()
{

}

TextureComponent::~TextureComponent()
{
}

HRESULT TextureComponent::CreateTexture(wchar_t* filepath, ID3D11ShaderResourceView** texture, RenderCommands* render_command)
{
    return CreateDDSTextureFromFile(render_command->GetDevice(), filepath, nullptr, texture);
}


void TextureComponent::BindTextures(int startSlot, int count, std::vector<ID3D11ShaderResourceView*> textures, RenderCommands* render_command)
{
    render_command->GetDeviceContext()->PSSetShaderResources(startSlot, count, &textures[0]);
}

