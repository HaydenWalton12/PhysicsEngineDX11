#include "Application.h"

#include <time.h>
#include <chrono>
#include <thread>

#define FPS60 0.016f

static bool gIsInitialized(false);
static unsigned __int64 gTicksPerSecond;
static unsigned __int64 gStartTicks;

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

int GetTimeMicroseconds() {
    if (false == gIsInitialized) {
        gIsInitialized = true;

        // Get the high frequency counter's resolution
        QueryPerformanceFrequency((LARGE_INTEGER*)&gTicksPerSecond);

        // Get the current time
        QueryPerformanceCounter((LARGE_INTEGER*)&gStartTicks);

        return 0;
    }

    unsigned __int64 tick;
    QueryPerformanceCounter((LARGE_INTEGER*)&tick);

    const double ticks_per_micro = (double)(gTicksPerSecond / 1000000);

    const unsigned __int64 timeMicro = (unsigned __int64)((double)(tick - gStartTicks) / ticks_per_micro);
    return (int)timeMicro;
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
    static int timeLastFrame = 0;
    static int numSamples = 0;
    static float avgTime = 0.0f;
    static float maxTime = 0.0f;

    ImGui_ImplDX11_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();


    //Checks For Scene Input 
    //Potentially Switch Scene At This Point
    Input();

    int time = GetTimeMicroseconds();
    float dt_us = (float)time - (float)timeLastFrame;
    if (dt_us < 16000.0f) {
        int x = 16000 - (int)dt_us;
        std::this_thread::sleep_for(std::chrono::microseconds(x));
        dt_us = 16000;
        time = GetTimeMicroseconds();
    }
    timeLastFrame = time;
    printf("\ndt_ms: %.1f    ", dt_us * 0.001f);
    
    // If the time is greater than 33ms (30fps)
        // then force the time difference to smaller
        // to prevent super large simulation steps.
    if (dt_us > 33000.0f) {
        dt_us = 33000.0f;
    }
    
    float dt_sec = dt_us * 0.001f * 0.001f;

    _pRenderCommands->ClearRenderTarget(_pDX11->_pRenderTargetView, _pDX11->_pDepthStencilView);

    //Get Scene Input
    _pCurrentScene->PollInput(dt_sec * 0.5f);
    int startTime = GetTimeMicroseconds();
    //Update Current Scene
    _pCurrentScene->Update(dt_sec * 0.5);
    int endTime = GetTimeMicroseconds();

    dt_us = (float)endTime - (float)startTime;
    if (dt_us > maxTime) {
        maxTime = dt_us;
    }

    avgTime = (avgTime * float(numSamples) + dt_us) / float(numSamples + 1);
    numSamples++;

    printf("frame dt_ms: %.2f %.2f %.2f", avgTime * 0.001f, maxTime * 0.001f, dt_us * 0.001f);


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

