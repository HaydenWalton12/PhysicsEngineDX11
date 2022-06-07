#include "Application.h"
#define FPS60 0.016f

extern LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
//Callback function , processes message sent to the window.
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    PAINTSTRUCT ps;
    HDC hdc;

    if (ImGui_ImplWin32_WndProcHandler(hWnd, message, wParam, lParam))
    {
        return true;
    }

    switch (message)
    {
    case WM_PAINT:

        hdc = BeginPaint(hWnd, &ps);
        EndPaint(hWnd, &ps);
        break;

    case WM_DESTROY:
        PostQuitMessage(0);
        break;

    default:
        return DefWindowProc(hWnd, message, wParam, lParam);

    }

    return 0;
}

//Initalises Window Coordinates , Projection Matrix & View Matrix 
HRESULT Application::Initialise(HINSTANCE hInstance, int nCmdShow)
{

    HRESULT hr = S_OK;
    if (FAILED(InitialiseWindow(hInstance, nCmdShow)))
    {

        return E_FAIL;

    }

    RECT rc;
    GetClientRect(_hWnd, &rc);

    //Pass calulated values , gives us window coordinates
    _WindowWidth = rc.right - rc.left;
    _WindowHeight = rc.bottom - rc.top;

    _pDX11 = new DX(_WindowWidth, _WindowHeight, _hWnd);
    _Tex = new TextureComponent();

    //InitialiseDevice , Assist creating core graphical components.
    _pDX11->InitialiseDevice();


    _pRenderCommands = new RenderCommands(_pDX11->_pDevice, _pDX11->_pDeviceContext, _pDX11->_pConstantBuffer);


    //Setup ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    ImGui_ImplWin32_Init(_hWnd);
    ImGui_ImplDX11_Init(_pDX11->_pDevice, _pDX11->_pDeviceContext);
    ImGui::StyleColorsDark();

    _pScene0 = new Scene0(_hWnd, _pRenderCommands, _Tex, _pGUIManager, _pDX11->_pDevice);
   

    _pSceneManager = new SceneManager(_pRenderCommands, _pGUIManager, _pDX11->_pDevice);
    //Initial Scene
    _pCurrentScene = _pScene0;

    return S_OK;
}

HRESULT Application::InitialiseWindow(HINSTANCE hInstance, int nCmdShow)
{
    //Register/Window class initialisation
    WNDCLASSEX wcex;

    //Register/Window Class definition
    wcex.cbSize = sizeof(WNDCLASSEX);
    wcex.style = CS_HREDRAW | CS_VREDRAW;
    wcex.lpfnWndProc = WndProc;
    wcex.cbClsExtra = 0;
    wcex.cbWndExtra = 0;
    wcex.hInstance = hInstance;
    wcex.hIcon = LoadIcon(hInstance, (LPCTSTR)IDI_TUTORIAL1);
    wcex.hCursor = LoadCursor(NULL, NULL);
    wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    wcex.lpszMenuName = nullptr;
    wcex.lpszClassName = L"TutorialWindowClass";
    wcex.hIconSm = LoadIcon(wcex.hInstance, (LPCTSTR)IDI_TUTORIAL1);


    //Check if class registration was correct
    if (!RegisterClassEx(&wcex))
    {
        return E_FAIL;
    }


    // Create window
    _hInst = hInstance;

    //Define window width/hight
    RECT rc = { 0, 0, 1280, 960 };
    AdjustWindowRect(&rc, WS_OVERLAPPEDWINDOW, FALSE);
    //Create Window
    _hWnd = CreateWindow(L"TutorialWindowClass", L"DX11 Framework", WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, rc.right - rc.left, rc.bottom - rc.top, nullptr, nullptr, hInstance, nullptr);

    //Check if Window did create.
    if (!_hWnd)
    {
        return E_FAIL;
    }

    ShowWindow(_hWnd, nCmdShow);

    DirectInput8Create(hInstance, DIRECTINPUT_VERSION, IID_IDirectInput8, (void**)&DirectInput, NULL);

    //Create Keyboard Device
    DirectInput->CreateDevice(GUID_SysKeyboard, &DIKeyBoard, NULL);

    //Create Mouse Device
    DirectInput->CreateDevice(GUID_SysMouse, &DIMouse, NULL);

    //Input format of device
    DIKeyBoard->SetDataFormat(&c_dfDIKeyboard);
    DIKeyBoard->SetCooperativeLevel(NULL, DISCL_FOREGROUND | DISCL_NONEXCLUSIVE);

    //Input format of mouse device
    DIMouse->SetDataFormat(&c_dfDIMouse);
    DIMouse->SetCooperativeLevel(NULL, DISCL_EXCLUSIVE | DISCL_FOREGROUND);


    return S_OK;
}

Application::Application()
{
}

Application::~Application()
{
    Cleanup();
}


void Application::RenderFrame()
{

    ImGui_ImplDX11_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();


    //Checks For Scene Input 
    //Potentially Switch Scene At This Point
    Input();





    _pRenderCommands->ClearRenderTarget(_pDX11->_pRenderTargetView, _pDX11->_pDepthStencilView);

    //Get Scene Input
    _pCurrentScene->PollInput(0.0016);

    //Update Current Scene
    _pCurrentScene->Update(0.0016);

    //Render IMGUI
    ImGui::Render();
    ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

    //Switch Draw Buffers In SwapChain
    _pRenderCommands->SwapChainPresent(_pDX11->_pSwapChain);
}


void Application::SwitchScene(SceneManager* scene)
{
    //if we have a current level exit it
    if (_pCurrentScene != nullptr) _pCurrentScene->ExitScene();
    _pCurrentScene = nullptr;

    //load the new level
    _pCurrentScene = scene;
    //reset the new level

}

void Application::Input()
{
    DIMOUSESTATE mouseCurrState;

    BYTE keyboardState[256];

    DIKeyBoard->Acquire();
    DIMouse->Acquire();

    DIMouse->GetDeviceState(sizeof(DIMOUSESTATE), &mouseCurrState);
    DIKeyBoard->GetDeviceState(sizeof(keyboardState), (LPVOID)&keyboardState);
    if (keyboardState[DIK_0] & 0x80)
    {
        _pCurrentScene->ResetScene();
        SwitchScene(_pScene0);
    }
    if (keyboardState[DIK_R] & 0x80)
    {
        _pCurrentScene->ResetScene();
    }

}

void Application::Cleanup()
{
    delete(_pDX11);
    delete(_pRenderCommands);



    delete(_Tex);
    delete(_pPixelShader);
    delete(_pVertexShader);
    delete(_PS);
    delete(_VS);


    _CameraObjects.clear();

}

