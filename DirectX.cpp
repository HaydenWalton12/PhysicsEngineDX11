
#include "DirectX.h"
DX::DX(UINT width, UINT height, HWND hWnd) : _WindowWidth(width), _WindowHeight(height), _hWnd(hWnd)
{
    _driverType = D3D_DRIVER_TYPE_NULL;
    _featureLevel = D3D_FEATURE_LEVEL_11_0;
}
DX::~DX()
{
    Cleanup();
}

void DX::InitialiseDevice()
{
    InitialiseSwapchain();
    InitialiseDepth();
    InitialiseRenderTarget();
    InitialiseViewport();
    InitialiseConstantBuffer();
    InitialiseSolid();
    InitialiseWireFrame();
    InitialiseSampler();
    InitialiseAlphaBlending();

}
void DX::Cleanup()
{
    delete(_pDeviceContext);
    delete(_pConstantBuffer);
    delete(_pRenderTargetView);
    delete(_pSwapChain);
    delete(_pDeviceContext);
    delete(_pDevice);
    delete(_pDepthStencilView);
    delete(_pDepthStencilBuffer);
    delete(_SolidRasterState);
}

HRESULT DX::InitialiseSwapchain()
{
    HRESULT hr = S_OK;

    UINT createDeviceFlags = 0;

#ifdef _DEBUG
    createDeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;
#endif
    //Lists drivetypes (methods of communicating to application to hardware)
    D3D_DRIVER_TYPE driverTypes[] =
    {
        D3D_DRIVER_TYPE_HARDWARE,
        D3D_DRIVER_TYPE_WARP,
        D3D_DRIVER_TYPE_REFERENCE,
    };

    UINT numDriverTypes = ARRAYSIZE(driverTypes);

    //Describes the DX11 versions used
    D3D_FEATURE_LEVEL featureLevels[] =
    {
        D3D_FEATURE_LEVEL_11_0,
        D3D_FEATURE_LEVEL_10_1,
        D3D_FEATURE_LEVEL_10_0,
    };


    UINT numFeatureLevels = ARRAYSIZE(featureLevels);

    //Create Swap Chain Description
    DXGI_SWAP_CHAIN_DESC sd;
    ZeroMemory(&sd, sizeof(sd));

    //Describes Swap Chain
    sd.BufferCount = 1;
    sd.BufferDesc.Width = _WindowWidth;
    sd.BufferDesc.Height = _WindowHeight;
    sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    sd.BufferDesc.RefreshRate.Numerator = 60;
    sd.BufferDesc.RefreshRate.Denominator = 1;
    sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.OutputWindow = _hWnd;
    sd.SampleDesc.Count = 1;
    sd.SampleDesc.Quality = 0;
    sd.Windowed = true;

    for (UINT driverTypeIndex = 0; driverTypeIndex < numDriverTypes; driverTypeIndex++)
    {
        _driverType = driverTypes[driverTypeIndex];
        hr = D3D11CreateDeviceAndSwapChain(nullptr, _driverType, nullptr, createDeviceFlags, featureLevels, numFeatureLevels,
            D3D11_SDK_VERSION, &sd, &_pSwapChain, &_pDevice, &_featureLevel, &_pDeviceContext);
        if (SUCCEEDED(hr))
            break;
    }
    if (FAILED(hr))
        return hr;
}
void DX::InitialiseSampler()
{
    // Create the sample state
    D3D11_SAMPLER_DESC sampDesc;

    ZeroMemory(&sampDesc, sizeof(sampDesc));

    sampDesc.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
    sampDesc.AddressU = D3D11_TEXTURE_ADDRESS_WRAP;
    sampDesc.AddressV = D3D11_TEXTURE_ADDRESS_WRAP;
    sampDesc.AddressW = D3D11_TEXTURE_ADDRESS_WRAP;
    sampDesc.ComparisonFunc = D3D11_COMPARISON_NEVER;
    sampDesc.MinLOD = 0;
    sampDesc.MaxLOD = D3D11_FLOAT32_MAX;

    _pDevice->CreateSamplerState(&sampDesc, &_pSamplerLinear);
    _pDeviceContext->PSSetSamplers(0, 1, &_pSamplerLinear);

}
void DX::InitialiseDepth()
{
    //Creates Depth Stencil buffer descriptor
    D3D11_TEXTURE2D_DESC depthStencilDesc;
    //Describing buffer descriptor
    depthStencilDesc.Width = _WindowWidth;
    depthStencilDesc.Height = _WindowHeight;
    depthStencilDesc.MipLevels = 1;
    depthStencilDesc.ArraySize = 1;
    depthStencilDesc.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
    depthStencilDesc.SampleDesc.Count = 1;
    depthStencilDesc.SampleDesc.Quality = 0;
    depthStencilDesc.Usage = D3D11_USAGE_DEFAULT;
    depthStencilDesc.BindFlags = D3D11_BIND_DEPTH_STENCIL;
    depthStencilDesc.CPUAccessFlags = 0;
    depthStencilDesc.MiscFlags = 0;

    //Creating depth/stencil buffer
    _pDevice->CreateTexture2D(&depthStencilDesc, nullptr, &_pDepthStencilBuffer);//Depth stencil buffer
    _pDevice->CreateDepthStencilView(_pDepthStencilBuffer, nullptr, &_pDepthStencilView);//Depth stencil view
}
HRESULT DX::InitialiseRenderTarget()
{
    HRESULT hr;
    // Create a render target view
    ID3D11Texture2D* pBackBuffer = nullptr;
    hr = _pSwapChain->GetBuffer(0, __uuidof(ID3D11Texture2D), (LPVOID*)&pBackBuffer);

    if (FAILED(hr))
        return hr;

    //Describes back buffer
    hr = _pDevice->CreateRenderTargetView(pBackBuffer, nullptr, &_pRenderTargetView);
    pBackBuffer->Release();

    if (FAILED(hr))
        return hr;

    //Changed it from nullptr to "_depthStencilView" cause now there is a depth/stencil view.
    _pDeviceContext->OMSetRenderTargets(1, &_pRenderTargetView, _pDepthStencilView);
    // Set primitive topology - Determines the format of how we draw primitives onto our DX11 Scene
    _pDeviceContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

}
void DX::InitialiseViewport()
{
    // Setup the viewport
    D3D11_VIEWPORT viewport;
    viewport.Width = (FLOAT)_WindowWidth;
    viewport.Height = (FLOAT)_WindowHeight;
    viewport.MinDepth = 0.0f;
    viewport.MaxDepth = 1.0f;
    viewport.TopLeftX = 0;
    viewport.TopLeftY = 0;
    _pDeviceContext->RSSetViewports(1, &viewport);
}
void DX::InitialiseConstantBuffer()
{
    // Create the constant buffer
    D3D11_BUFFER_DESC constantbufferdescription;
    ZeroMemory(&constantbufferdescription, sizeof(constantbufferdescription));
    //Describe Constant Buffer
    constantbufferdescription.Usage = D3D11_USAGE_DEFAULT;
    constantbufferdescription.ByteWidth = sizeof(ConstantBuffer);
    constantbufferdescription.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
    constantbufferdescription.CPUAccessFlags = 0;

    //Create Buffer - Using Description above , 3rd parameters assigns value to Constantbuffer buffer pointer.
    _pDevice->CreateBuffer(&constantbufferdescription, nullptr, &_pConstantBuffer);

    _pDeviceContext->VSSetConstantBuffers(0, 1, &_pConstantBuffer);
    _pDeviceContext->PSSetConstantBuffers(0, 1, &_pConstantBuffer);
}



void DX::InitialiseSolid()
{
    //Create wireframe description
    D3D11_RASTERIZER_DESC solid;
    ZeroMemory(&solid, sizeof(D3D11_RASTERIZER_DESC));

    //Describe Wireframe
    solid.FillMode = D3D11_FILL_SOLID;
    solid.CullMode = D3D11_CULL_NONE;

    //Create wirefram rasterizer stage
    _pDevice->CreateRasterizerState(&solid, &_SolidRasterState);
}
void DX::InitialiseWireFrame()
{
    //Create wireframe description
    D3D11_RASTERIZER_DESC wireframe;
    ZeroMemory(&wireframe, sizeof(D3D11_RASTERIZER_DESC));

    //Describe Wireframe
    wireframe.FillMode = D3D11_FILL_WIREFRAME;
    wireframe.CullMode = D3D11_CULL_NONE;

    //Create wirefram rasterizer stage
    _pDevice->CreateRasterizerState(&wireframe, &_WireFrameRasterState);
}

void DX::InitialiseAlphaBlending()
{
    D3D11_BLEND_DESC blendingdescription;

    ZeroMemory(&blendingdescription, sizeof(blendingdescription));

    D3D11_RENDER_TARGET_BLEND_DESC rtbd;

    ZeroMemory(&rtbd, sizeof(rtbd));

    rtbd.BlendEnable = true;
    rtbd.SrcBlend = D3D11_BLEND_SRC_COLOR;
    rtbd.DestBlend = D3D11_BLEND_BLEND_FACTOR;
    rtbd.BlendOp = D3D11_BLEND_OP_ADD;
    rtbd.SrcBlendAlpha = D3D11_BLEND_ONE;
    rtbd.DestBlendAlpha = D3D11_BLEND_ZERO;
    rtbd.BlendOpAlpha = D3D11_BLEND_OP_ADD;
    rtbd.RenderTargetWriteMask = D3D10_COLOR_WRITE_ENABLE_ALL;

    blendingdescription.AlphaToCoverageEnable = false;
    blendingdescription.RenderTarget[0] = rtbd;

    _pDevice->CreateBlendState(&blendingdescription, &_BlendState);

    float blendFactor[] = { 0.75f, 0.75f, 0.75f, 1.0f };



}