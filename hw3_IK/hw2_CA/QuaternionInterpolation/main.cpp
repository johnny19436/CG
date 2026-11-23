/*
Computer animation assignment 2: Inverse Kinematics

First step: Search TODO comments to find the methods that you need to implement.
    - src/simulation/kinematics.cpp
*/
#include <algorithm>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

#include "Eigen/Core"
#include "GLFW/glfw3.h"
#define GLAD_GL_IMPLEMENTATION
#include "glad/gl.h"
#undef GLAD_GL_IMPLEMENTATION
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "acclaim.h"
#include "graphics.h"
#include "simulation.h"
#include "util.h"


// Global variables are evil!
namespace {
// Shadow texture size, default is 2048 * 2048
int shadowTextureSize = 2048;
// Scene size
int g_ScreenWidth = 1024, g_ScreenHeight = 768;
// Camera controlled by panel
graphics::DefaultCamera defaultCamera;
// Camera controlled by keyboard and mouse
graphics::FreeCamera freeCamera;
// Current camera
graphics::Camera* currentCamera = &defaultCamera;
// Switch for render camera control panel
bool isUsingCameraPanel = false;
// Is free camera?
bool isUsingFreeCamera = false;
// Mouse is disabled?
bool isMouseBinded = false;
// Animation playback state
bool isSimulating = false;
int currentFrame = 0;
int totalFrames = 0;
// Available animation clips
const char* animations[] = {"walk.amc", "acrobatics.amc", "gymnastics.amc", "running.amc", "shoot.amc", "jump.amc"};
static const char* current_animation = animations[0];
bool isChanged = false;
// FK-only motion object
std::unique_ptr<acclaim::Motion> fkAnimation;
// Acclaim assets folder, kept global so mainPanel can create new skeletons
util::fs::path acclaim_folder;
// Number of frames between consecutive keyframes for interpolation
int keyframeInterval = 10;
// Whether the current animation is being played as interpolated
bool isInterpolatedAnimation = false;
// Rotation interpolation method: true = Squad, false = Slerp
bool useSquad = false;
// Root position interpolation method: true = linear, false = Catmull-Bezier
bool useLinearRoot = false;
// Overlay a ghost skeleton at the next keyframe position
bool showNextKeyframe = false;

// Keyframe data built once when "Start Interpolation" is clicked.
// All rotations are stored as quaternions (hemisphere-normalised globally)
// to avoid Euler round-trip artefacts (angle jump, gimbal lock).
struct KeyframeData {
    KeyframeData() = default;
    std::vector<int> indices;
    std::vector<std::vector<Eigen::Quaterniond>> rotations;       // [kf][bone]
    std::vector<Eigen::Vector4d>                 rootPositions;   // [kf]
    std::vector<std::vector<Eigen::Vector4d>>    boneTranslations;// [kf][bone]
    util::RotationInterpQuat rotInterp;
    util::PositionInterp     posInterp;
};
KeyframeData kfData;
}  // namespace

/**
 * @brief When resizing window, we need to update viewport and camera's aspect
 * ratio.
 * @param window current GLFW context
 * @param screenWidth Screen width after resize.
 * @param screenHeight Screen height after resize.
 */
void reshape(GLFWwindow* window, int screenWidth, int screenHeight);

/**
 * @brief Initialize globals and OpenGL
 *
 * @return `GLFWwindow*` The current GLFW context window
 */
GLFWwindow* initialize();
/**
 * @brief Callback function for the debug camera.
 */
void cameraKeyboard(GLFWwindow* window, int key, int scancode, int action, int mode);
/**
 * @brief Reset pointer of std::unique_ptr to call destructor before calling
 * glfwTerminate() to avoid segfault since some destructor contains OpenGL
 * functions.
 *
 * @param window The context to be destroyed
 */
void shutdown();
// Forward declarations for pipeline helpers defined later in this file
void buildKeyframeData(const std::vector<acclaim::Posture>& src, int interval,
                       util::RotationInterpQuat rotInterp, util::PositionInterp posInterp);
kinematics::FrameQuat interpolateFrame(const KeyframeData& kd, int frame);

/**
 * @brief Dear-ImGui main control panel
 *
 */
void mainPanel();
/**
 * @brief Dear-ImGui camera control panel
 *
 * @param window: Current context
 */
void cameraPanel(GLFWwindow* window);
/**
 * @brief Put panels between ImGui::NewFrame() and ImGui::Render() to render
 *
 * @param window: Current context
 */
void renderUI(GLFWwindow* window);

int main() {
    GLFWwindow* window = initialize();
    // No window created
    if (window == nullptr) return 1;
    // Shader programs
    graphics::Program renderProgram;
    graphics::Program skyboxRenderProgram;
    graphics::Program shadowProgram;
    // Texture for shadow mapping
    graphics::ShadowMapTexture shadow(shadowTextureSize);
    // The skybox
    graphics::Box skybox;
    // Ground
    graphics::Plane plane;

    // Load assets, setup textures
    {
        // Shader
        auto shader_folder = util::PathFinder::find("Shader");
        graphics::Shader shadowVertexShader(shader_folder / "shadow.vert", GL_VERTEX_SHADER);
        graphics::Shader shadowFragmentShader(shader_folder / "shadow.frag", GL_FRAGMENT_SHADER);
        graphics::Shader renderVertexShader(shader_folder / "render.vert", GL_VERTEX_SHADER);
        graphics::Shader renderFragmentShader(shader_folder / "render.frag", GL_FRAGMENT_SHADER);
        graphics::Shader skyboxVertexShader(shader_folder / "skybox.vert", GL_VERTEX_SHADER);
        graphics::Shader skyboxFragmentShader(shader_folder / "skybox.frag", GL_FRAGMENT_SHADER);
        // Setup shaders, these objects can be destroyed after linkShader()
        renderProgram.attachLinkShader(renderVertexShader, renderFragmentShader);
        shadowProgram.attachLinkShader(shadowVertexShader, shadowFragmentShader);
        skyboxRenderProgram.attachLinkShader(skyboxVertexShader, skyboxFragmentShader);
        // Texture
        auto texture_folder = util::PathFinder::find("Texture");
        auto wood = std::make_shared<graphics::Texture>(texture_folder / "wood.png");
        plane.setTexture(std::move(wood));

        auto sky = std::make_shared<graphics::CubeTexture>(std::array<util::fs::path, 6>{
            texture_folder / "skybox0.png", texture_folder / "skybox1.png", texture_folder / "skybox2.png",
            texture_folder / "skybox3.png", texture_folder / "skybox4.png", texture_folder / "skybox5.png"});
        skybox.setTexture(std::move(sky));
        // Acclaim
        acclaim_folder = util::PathFinder::find("Acclaim");
        // Load initial FK animation (walk)
        auto fkSkeleton = std::make_unique<acclaim::Skeleton>(acclaim_folder / "skeleton.asf", 0.2);
        fkAnimation = std::make_unique<acclaim::Motion>(acclaim_folder / current_animation, std::move(fkSkeleton));
        totalFrames = fkAnimation->getFrameNum();
    }
    // Setup light, uniforms are persisted.
    {
        Eigen::Vector4f lightPosition(11.1f, 24.9f, -14.8f, 0.0f);
        Eigen::Matrix4f lightSpaceMatrix = util::ortho(-30.0f, 30.0f, -30.0f, 30.0f, -75.0f, 75.0f);
        lightSpaceMatrix *= util::lookAt(lightPosition, Eigen::Vector4f::Zero(), Eigen::Vector4f::UnitY());
        // Shader program should be use atleast once before setting up uniforms
        shadowProgram.use();
        shadowProgram.setUniform("lightSpaceMatrix", lightSpaceMatrix);

        renderProgram.use();
        renderProgram.setUniform("lightSpaceMatrix", lightSpaceMatrix);
        renderProgram.setUniform("shadowMap", shadow.getIndex());
        renderProgram.setUniform("lightPos", lightPosition);
    }
    while (!glfwWindowShouldClose(window)) {
        // Moving camera only if debug camera is on.
        if (isUsingFreeCamera) {
            if (isMouseBinded) freeCamera.moveSight(window);
            freeCamera.moveCamera(window);
        }
        currentCamera->update();

        // FK frame advance
        if (isSimulating) {
            ++currentFrame;
            currentFrame %= totalFrames;
        }

        // Animation clip switch
        if (isChanged) {
            auto sk = std::make_unique<acclaim::Skeleton>(acclaim_folder / "skeleton.asf", 0.2);
            fkAnimation = std::make_unique<acclaim::Motion>(acclaim_folder / current_animation, std::move(sk));
            totalFrames = fkAnimation->getFrameNum();
            currentFrame = 0;
            isSimulating = false;
            isChanged = false;
            isInterpolatedAnimation = false;
            kfData = KeyframeData();
        }

        // FK solve — three explicit steps in interpolation mode
        if (isInterpolatedAnimation) {
            // Step 1: Interpolation
            //   Compute quaternion rotations and root position for this frame
            //   from the pre-built keyframe data. Entirely in quaternion space.
            kinematics::FrameQuat frame = interpolateFrame(kfData, currentFrame);

            // Step 2: Local Transform  +  Step 3: Forward Kinematics
            //   forwardSolver applies rot_parent_current (local transform)
            //   then propagates the parent chain (FK) — see kinematics.cpp.
            //   No Euler conversion occurs, eliminating angle jump and gimbal lock.
            kinematics::forwardSolver(frame, fkAnimation->getSkeleton()->getBonePointer(0));
            fkAnimation->getSkeleton()->setModelMatrices();
        } else {
            // Original path: Euler angles from AMC file, standard FK solver.
            fkAnimation->forwardkinematics(currentFrame);
        }

        // Highlight keyframe positions: cyan bones on a keyframe, default yellow otherwise.
        // Only active while playing an interpolated animation so there is something to mark.
        if (isInterpolatedAnimation) {
        bool onKeyframe = std::find(kfData.indices.begin(), kfData.indices.end(), currentFrame)
                          != kfData.indices.end();
        fkAnimation->getSkeleton()->setBoneColor(onKeyframe
            ? Eigen::Vector4f(0.2f, 0.8f, 1.0f, 1.0f)   // cyan  – keyframe
            : Eigen::Vector4f(0.6f, 0.6f, 0.0f, 1.0f)); // yellow – interpolated
    }

    // 1. Render shadow to texture
        glViewport(0, 0, shadow.getShadowSize(), shadow.getShadowSize());
        glCullFace(GL_FRONT);
        shadowProgram.use();
        shadow.bindFrameBuffer();
        glClear(GL_DEPTH_BUFFER_BIT);
        plane.render(&shadowProgram);
        fkAnimation->render(&shadowProgram);
        shadow.unbindFrameBuffer();
        glCullFace(GL_BACK);
        // 2. Render scene
        glViewport(0, 0, g_ScreenWidth, g_ScreenHeight);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderProgram.use();
        renderProgram.setUniform("viewPos", currentCamera->getPosition());
        renderProgram.setUniform("VP", currentCamera->getViewWithProjectionMatrix());

        plane.render(&renderProgram);
        fkAnimation->render(&renderProgram);
        // Ghost skeleton at the next keyframe (no shadow — ghosts don't cast shadows).
        if (isInterpolatedAnimation && showNextKeyframe && !kfData.indices.empty()) {
            // Find the first keyframe index strictly after the current frame.
            int nextKFseg = -1;
            for (int i = 0; i < static_cast<int>(kfData.indices.size()); ++i) {
                if (kfData.indices[i] > currentFrame) { nextKFseg = i; break; }
            }
            if (nextKFseg != -1) {
                kinematics::FrameQuat ghostFrame;
                ghostFrame.boneRotations    = kfData.rotations[nextKFseg];
                ghostFrame.rootTranslation  = kfData.rootPositions[nextKFseg];
                ghostFrame.boneTranslations = kfData.boneTranslations[nextKFseg];
                kinematics::forwardSolver(ghostFrame, fkAnimation->getSkeleton()->getBonePointer(0));
                fkAnimation->getSkeleton()->setModelMatrices();
                fkAnimation->getSkeleton()->setBoneColor(Eigen::Vector4f(0.9f, 0.4f, 0.1f, 1.0f));  // orange
                fkAnimation->render(&renderProgram);
            }
        }
        // 3. Render the skybox .
        skyboxRenderProgram.use();
        skyboxRenderProgram.setUniform("projection", currentCamera->getProjectionMatrix());
        skyboxRenderProgram.setUniform("view", currentCamera->getViewMatrix());
        skybox.render(&skyboxRenderProgram);
        // 4. Render ImGui UI
        renderUI(window);

        glFlush();
        glfwSwapBuffers(window);
        // Keyboard and mouse inputs.
        glfwPollEvents();
    }
    shutdown();
    glfwDestroyWindow(window);
    return 0;
}

void reshape(GLFWwindow*, int screenWidth, int screenHeight) {
    g_ScreenWidth = screenWidth;
    g_ScreenHeight = screenHeight;
    glViewport(0, 0, g_ScreenWidth, g_ScreenHeight);
    defaultCamera.setAspectRatio(g_ScreenWidth, g_ScreenHeight);
    freeCamera.setAspectRatio(g_ScreenWidth, g_ScreenHeight);
}

GLFWwindow* initialize() {
    // Initialize GLFW
    if (glfwInit() == GLFW_FALSE) {
        std::cerr << "Failed to initialize GLFW!" << std::endl;
        return nullptr;
    }
    std::atexit(glfwTerminate);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    // Create GLFW context
    GLFWwindow* window = glfwCreateWindow(g_ScreenWidth, g_ScreenHeight, "Inverse Kinematics", nullptr, nullptr);
    if (window == nullptr) {
        std::cerr << "Failed to create OpenGL 4.1 window!" << std::endl;
        return nullptr;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    // Initialize glad
    if (!gladLoadGL(glfwGetProcAddress)) {
        std::cerr << "Failed to initialize OpenGL context" << std::endl;
        return nullptr;
    }
    // Find assets folder
    if (!util::PathFinder::initialize()) {
        std::cerr << "Cannot find assets!" << std::endl;
        return nullptr;
    }
    // OK, everything works fine
    // ----------------------------------------------------------
    // For high dpi monitors
    glfwGetFramebufferSize(window, &g_ScreenWidth, &g_ScreenHeight);
    GLFWmonitor* moniter = glfwGetPrimaryMonitor();
    const GLFWvidmode* vidMode = glfwGetVideoMode(moniter);
    int maxTextureSize = 1024;
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxTextureSize);
    shadowTextureSize = std::min(shadowTextureSize, maxTextureSize);
    // Print some system information
    std::cout << std::left << std::setw(26) << "Current OpenGL renderer"
              << ": " << glGetString(GL_RENDERER) << std::endl;
    std::cout << std::left << std::setw(26) << "Current OpenGL context"
              << ": " << glGetString(GL_VERSION) << std::endl;
    std::cout << std::left << std::setw(26) << "Moniter refresh rate"
              << ": " << vidMode->refreshRate << " Hz" << std::endl;
    std::cout << std::left << std::setw(26) << "Max texture size support"
              << ": " << maxTextureSize << " * " << maxTextureSize << std::endl;
    std::cout << std::left << std::setw(26) << "Shadow texture size"
              << ": " << shadowTextureSize << " * " << shadowTextureSize << std::endl;
    // Setup Opengl
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    // Setup GLFW
    reshape(window, g_ScreenWidth, g_ScreenHeight);
    glfwSetFramebufferSizeCallback(window, reshape);
    // Initialize dear-ImGui
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 410 core");
    return window;
}

void cameraKeyboard(GLFWwindow* window, int key, int, int action, int) {
    if (action == GLFW_PRESS && key == GLFW_KEY_F9) {
        if (glfwGetInputMode(window, GLFW_CURSOR) == GLFW_CURSOR_DISABLED) {
            // Show the mouse cursor
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            isMouseBinded = false;
        } else {
            // Reset delta x and delta y to avoid view teleporting
            freeCamera.reset();
            // Hide the mouse cursor
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            isMouseBinded = true;
        }
    }
}

void shutdown() {
    fkAnimation.reset();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

// Build quaternion keyframe data from the source postures.
// Posture::bone_rotations already holds unit quaternions (converted from Euler at load time).
// This function samples keyframes at the given interval, then hemisphere-normalises globally
// across the full keyframe chain so that every consecutive pair has a positive dot product.
// The single global pass eliminates per-segment hemisphere flips that would cause angle jumps.
void buildKeyframeData(const std::vector<acclaim::Posture>& src, int interval,
                       util::RotationInterpQuat rotInterp, util::PositionInterp posInterp) {
    kfData = KeyframeData();
    kfData.rotInterp = rotInterp;
    kfData.posInterp = posInterp;

    int total    = static_cast<int>(src.size());
    int numBones = static_cast<int>(src[0].bone_rotations.size());

    // Sample keyframe indices; always include the last frame.
    for (int i = 0; i < total; i += interval) kfData.indices.push_back(i);
    if (kfData.indices.back() != total - 1) kfData.indices.push_back(total - 1);
    int nkf = static_cast<int>(kfData.indices.size());

    kfData.rotations.resize(nkf, std::vector<Eigen::Quaterniond>(numBones));
    kfData.rootPositions.resize(nkf);
    kfData.boneTranslations.resize(nkf);

    for (int ki = 0; ki < nkf; ++ki) {
        const acclaim::Posture& p = src[kfData.indices[ki]];
        // bone_rotations are already unit quaternions (converted in readAMCFile).
        kfData.rotations[ki]        = p.bone_rotations;
        kfData.rootPositions[ki]    = p.bone_translations[0];
        kfData.boneTranslations[ki] = p.bone_translations;
    }

    for (int ki = 1; ki < nkf; ++ki) {
        // #TODO Part 3
        // Add Global hemisphere normalisation — walk the full keyframe chain once per
        // bone and flip any quaternion whose dot with its predecessor is different.
        ///////////////////////
        for (int boneIdx = 0; boneIdx < numBones; ++boneIdx) {
            const Eigen::Quaterniond& previousQuat = kfData.rotations[ki - 1][boneIdx];
            Eigen::Quaterniond& currentQuat = kfData.rotations[ki][boneIdx];
            if (previousQuat.dot(currentQuat) < 0.0) {
                currentQuat.coeffs() *= -1.0;
            }
            currentQuat.normalize();
        }
        ///////////////////////
        
    }
}

// Step 1 of the per-frame pipeline: compute interpolated FrameQuat for `frame`.
// The result is passed directly to forwardSolverQuat (Steps 2 & 3) without any
// Euler conversion, so there is no gimbal lock or angle jump on the FK path.
kinematics::FrameQuat interpolateFrame(const KeyframeData& kd, int frame) {
    int nkf      = static_cast<int>(kd.indices.size());
    int numBones = static_cast<int>(kd.rotations[0].size());

    // Locate the segment [seg, seg+1] that contains this frame.
    int seg = nkf - 2;
    for (int i = 0; i + 1 < nkf; ++i) {
        if (frame <= kd.indices[i + 1]) { seg = i; break; }
    }
    int fa = kd.indices[seg], fb = kd.indices[seg + 1];
    double t = (fb > fa) ? static_cast<double>(frame - fa) / (fb - fa) : 0.0;

    int i_km1 = std::max(0, seg - 1);
    int i_k2  = std::min(nkf - 1, seg + 2);

    kinematics::FrameQuat result;
    result.boneRotations.resize(numBones);

    // Rotation interpolation (quaternion-native, no Euler back-conversion).
    for (int b = 0; b < numBones; ++b) {
        result.boneRotations[b] = kd.rotInterp(
            kd.rotations[i_km1][b], kd.rotations[seg][b],
            kd.rotations[seg + 1][b], kd.rotations[i_k2][b], t);
    }

    // Root world position: Cubic Bezier (or whatever posInterp was chosen).
    result.rootTranslation = kd.posInterp(
        kd.rootPositions[i_km1], kd.rootPositions[seg],
        kd.rootPositions[seg + 1], kd.rootPositions[i_k2], t);

    // Non-root bone translations (nearly always zero).
    result.boneTranslations = kd.boneTranslations[seg];

    return result;
}

void mainPanel() {
    // Main Panel
    ImGui::SetNextWindowSize(ImVec2(330.0f, 290.0f), ImGuiCond_Once);
    ImGui::SetNextWindowCollapsed(0, ImGuiCond_Once);
    ImGui::SetNextWindowPos(ImVec2(60.0f, 525.0f), ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(0.2f);
    if (ImGui::Begin("Control")) {
        // Open / close camera panel
        if (ImGui::Button("Camera Panel")) {
            isUsingCameraPanel ^= true;
        }
        // Animation clip selector
            if (ImGui::BeginCombo("Animation", current_animation)) {
                for (const char* anim : animations) {
                    bool selected = (current_animation == anim);
                    if (ImGui::Selectable(anim, selected)) {
                        current_animation = anim;
                        isChanged = true;
                    }
                    if (selected) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
            // Animation source indicator
            if (isInterpolatedAnimation) {
                ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "[Interpolated]");
            } else {
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "[Original]");
            }
            // Frame scrubber
            ImGui::SliderInt("Current Frame", &currentFrame, 0, totalFrames - 1);
            // Playback controls
            if (ImGui::Button("Play")) isSimulating = true;
            ImGui::SameLine();
            if (ImGui::Button("Pause")) isSimulating = false;
            ImGui::SameLine();
            if (ImGui::Button("Stop")) {
                isSimulating = false;
                currentFrame = 0;
            }
            ImGui::Separator();
            ImGui::SliderInt("Keyframe Interval", &keyframeInterval, 1, 200);
            // Rotation method toggle
            if (ImGui::Button(useSquad ? "Rotation: Squad" : "Rotation: Slerp")) {
                useSquad = !useSquad;
                // Re-bake immediately so the label always matches what's playing.
                if (isInterpolatedAnimation) {
                    auto rotMethod = useSquad
                        ? util::RotationInterpQuat(util::InterpolationMath::squadQuat)
                        : util::RotationInterpQuat(util::InterpolationMath::slerpQuat);
                    auto posMethod = useLinearRoot
                        ? util::PositionInterp(util::InterpolationMath::RootPositionLinear)
                        : util::PositionInterp(util::InterpolationMath::RootPositionInterpolation);
                    buildKeyframeData(fkAnimation->getPostures(), keyframeInterval,
                                      rotMethod, posMethod);
                }
            }
            if (ImGui::Button(useLinearRoot ? "Position: Linear" : "Position: Catmull-Bezier")) {
                useLinearRoot = !useLinearRoot;
                // Re-bake immediately to reflect position method switch.
                if (isInterpolatedAnimation) {
                    auto rotMethod = useSquad
                        ? util::RotationInterpQuat(util::InterpolationMath::squadQuat)
                        : util::RotationInterpQuat(util::InterpolationMath::slerpQuat);
                    auto posMethod = useLinearRoot
                        ? util::PositionInterp(util::InterpolationMath::RootPositionLinear)
                        : util::PositionInterp(util::InterpolationMath::RootPositionInterpolation);
                    buildKeyframeData(fkAnimation->getPostures(), keyframeInterval,
                                      rotMethod, posMethod);
                }
            }
            if (isInterpolatedAnimation)
                ImGui::Checkbox("Show Next Keyframe", &showNextKeyframe);
            {
                if (ImGui::Button("Start Interpolation")) {
                    auto rotMethod = useSquad
                        ? util::RotationInterpQuat(util::InterpolationMath::squadQuat)
                        : util::RotationInterpQuat(util::InterpolationMath::slerpQuat);
                    auto posMethod = useLinearRoot
                        ? util::PositionInterp(util::InterpolationMath::RootPositionLinear)
                        : util::PositionInterp(util::InterpolationMath::RootPositionInterpolation);
                    buildKeyframeData(fkAnimation->getPostures(), keyframeInterval,
                                      rotMethod, posMethod);
                    currentFrame = 0;
                    isSimulating = false;
                    isInterpolatedAnimation = true;
                }
                ImGui::SameLine();
                if (ImGui::Button("Restore Original")) {
                    isChanged = true;
                }
            }
    }
    ImGui::End();
}

void cameraPanel(GLFWwindow* window) {
    // Camera control
    // Create imgui window
    ImGui::SetNextWindowSize(ImVec2(500.0f, 210.0f), ImGuiCond_Once);
    ImGui::SetNextWindowCollapsed(0, ImGuiCond_Once);
    ImGui::SetNextWindowPos(ImVec2(370.0f, 525.0f), ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(0.2f);
    if (ImGui::Begin("Camera", &isUsingCameraPanel)) {
        if (!isUsingFreeCamera) ImGui::Text("Use this panel to control the camera");
        Eigen::Vector4f cameraPosition = currentCamera->getPosition();
        ImGui::Text("Camera position : (%f, %f, %f)", cameraPosition[0], cameraPosition[1], cameraPosition[2]);
        Eigen::Vector4f cameraCenter = currentCamera->getCenter();
        ImGui::Text("Camera lookat : (%f, %f, %f)", cameraCenter[0], cameraCenter[1], cameraCenter[2]);
        if (!isUsingFreeCamera) {
            ImGui::SliderFloat("Camera rotation angle", defaultCamera.getCameraRotationAnglePointer(), 0.0f, 360.0f);
            ImGui::SliderFloat("Camera rotation radius", defaultCamera.getCameraRotationRadiusPointer(), 0.125f, 50.0f);
            ImGui::SliderFloat("Camera Y Offset", defaultCamera.getCameraYOffsetPointer(), -10.0f, 10.0f);
            if (ImGui::Button("Debug Mode")) {
                isUsingFreeCamera = true;
                currentCamera = &freeCamera;
                glfwSetKeyCallback(window, cameraKeyboard);
            }
        } else {
            ImGui::InputFloat("Mouse sensitivity", freeCamera.getMouseSensitivityPointer(), 0.01f, 0.05f);
            ImGui::InputFloat("Move speed", freeCamera.getMoveSpeedPointer(), 0.01f, 0.05f);
            ImGui::Text("Use W A S D CTRL SPACE to move");
            ImGui::Text("Press F9 to bind / unbind mouse");
            ImGui::Text("Bind mouse to control view");
            if (ImGui::Button("Leave debug Mode")) {
                isUsingFreeCamera = false;
                currentCamera = &defaultCamera;
                glfwSetCursorPosCallback(window, nullptr);
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            }
        }
    }
    ImGui::End();
}

void renderUI(GLFWwindow* window) {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    mainPanel();
    if (isUsingCameraPanel) cameraPanel(window);
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}
