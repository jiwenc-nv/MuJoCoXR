#include "vk_context.h"

#include <cstring>

#include "mxr_log.h"
#include "xr_shell.h"

namespace {

bool VkOk(VkResult r, const char* what) {
  if (r == VK_SUCCESS) {
    return true;
  }
  LOGE("%s failed: %d", what, r);
  return false;
}

}  // namespace

bool VkContext::Create(XrShell* xr) {
  XrGraphicsRequirementsVulkanKHR reqs;
  if (!xr->GetVulkanRequirements(&reqs)) {
    return false;
  }
  LOGI("runtime Vulkan API range: %llu..%llu",
       static_cast<unsigned long long>(reqs.minApiVersionSupported),
       static_cast<unsigned long long>(reqs.maxApiVersionSupported));

  VkApplicationInfo app_info{VK_STRUCTURE_TYPE_APPLICATION_INFO};
  app_info.pApplicationName = "mujocoxr";
  app_info.apiVersion = VK_API_VERSION_1_1;
  VkInstanceCreateInfo inst_info{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
  inst_info.pApplicationInfo = &app_info;
  if (!xr->CreateVulkanInstance(&inst_info, &instance_)) {
    return false;
  }
  if (!xr->GetVulkanPhysicalDevice(instance_, &physical_)) {
    return false;
  }

  uint32_t nqf = 0;
  vkGetPhysicalDeviceQueueFamilyProperties(physical_, &nqf, nullptr);
  std::vector<VkQueueFamilyProperties> qf(nqf);
  vkGetPhysicalDeviceQueueFamilyProperties(physical_, &nqf, qf.data());
  queue_family_ = UINT32_MAX;
  for (uint32_t i = 0; i < nqf; ++i) {
    if (qf[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) {
      queue_family_ = i;
      break;
    }
  }
  if (queue_family_ == UINT32_MAX) {
    LOGE("no graphics queue family");
    return false;
  }

  float prio = 1.0f;
  VkDeviceQueueCreateInfo queue_info{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
  queue_info.queueFamilyIndex = queue_family_;
  queue_info.queueCount = 1;
  queue_info.pQueuePriorities = &prio;
  VkPhysicalDeviceFeatures features{};
  VkDeviceCreateInfo dev_info{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
  dev_info.queueCreateInfoCount = 1;
  dev_info.pQueueCreateInfos = &queue_info;
  dev_info.pEnabledFeatures = &features;
  if (!xr->CreateVulkanDevice(physical_, &dev_info, &device_)) {
    return false;
  }
  vkGetDeviceQueue(device_, queue_family_, 0, &queue_);

  VkCommandPoolCreateInfo pool_info{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
  pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
  pool_info.queueFamilyIndex = queue_family_;
  if (!VkOk(vkCreateCommandPool(device_, &pool_info, nullptr, &command_pool_),
            "vkCreateCommandPool")) {
    return false;
  }
  VkCommandBufferAllocateInfo alloc{
      VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
  alloc.commandPool = command_pool_;
  alloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  alloc.commandBufferCount = 1;
  if (!VkOk(vkAllocateCommandBuffers(device_, &alloc, &command_buffer_),
            "vkAllocateCommandBuffers")) {
    return false;
  }
  VkFenceCreateInfo fence_info{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
  fence_info.flags = VK_FENCE_CREATE_SIGNALED_BIT;
  return VkOk(vkCreateFence(device_, &fence_info, nullptr, &frame_fence_),
              "vkCreateFence");
}

bool VkContext::InitRenderTargets(XrShell* xr) {
  color_format_ = static_cast<VkFormat>(xr->swapchain_format());

  // Depth format: prefer D32, fall back to D24S8.
  for (VkFormat f : {VK_FORMAT_D32_SFLOAT, VK_FORMAT_D24_UNORM_S8_UINT,
                     VK_FORMAT_D16_UNORM}) {
    VkFormatProperties props;
    vkGetPhysicalDeviceFormatProperties(physical_, f, &props);
    if (props.optimalTilingFeatures &
        VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT) {
      depth_format_ = f;
      break;
    }
  }
  if (depth_format_ == VK_FORMAT_UNDEFINED) {
    LOGE("no depth format");
    return false;
  }

  VkAttachmentDescription atts[2]{};
  atts[0].format = color_format_;
  atts[0].samples = VK_SAMPLE_COUNT_1_BIT;
  atts[0].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  atts[0].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
  atts[0].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
  atts[0].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  atts[0].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  atts[0].finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
  atts[1].format = depth_format_;
  atts[1].samples = VK_SAMPLE_COUNT_1_BIT;
  atts[1].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  atts[1].storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  atts[1].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
  atts[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  atts[1].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  atts[1].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

  VkAttachmentReference color_ref{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
  VkAttachmentReference depth_ref{
      1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};
  VkSubpassDescription subpass{};
  subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
  subpass.colorAttachmentCount = 1;
  subpass.pColorAttachments = &color_ref;
  subpass.pDepthStencilAttachment = &depth_ref;

  VkRenderPassCreateInfo rp_info{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
  rp_info.attachmentCount = 2;
  rp_info.pAttachments = atts;
  rp_info.subpassCount = 1;
  rp_info.pSubpasses = &subpass;
  if (!VkOk(vkCreateRenderPass(device_, &rp_info, nullptr, &render_pass_),
            "vkCreateRenderPass")) {
    return false;
  }

  const auto& swapchains = xr->swapchains();
  eyes_.resize(swapchains.size());
  for (size_t i = 0; i < swapchains.size(); ++i) {
    const auto& sc = swapchains[i];
    auto& eye = eyes_[i];
    eye.width = sc.width;
    eye.height = sc.height;

    VkImageCreateInfo di{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
    di.imageType = VK_IMAGE_TYPE_2D;
    di.format = depth_format_;
    di.extent = {static_cast<uint32_t>(sc.width),
                 static_cast<uint32_t>(sc.height), 1};
    di.mipLevels = 1;
    di.arrayLayers = 1;
    di.samples = VK_SAMPLE_COUNT_1_BIT;
    di.tiling = VK_IMAGE_TILING_OPTIMAL;
    di.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
    if (!VkOk(vkCreateImage(device_, &di, nullptr, &eye.depth_image),
              "vkCreateImage(depth)")) {
      return false;
    }
    VkMemoryRequirements mem_reqs;
    vkGetImageMemoryRequirements(device_, eye.depth_image, &mem_reqs);
    VkMemoryAllocateInfo mem_info{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    mem_info.allocationSize = mem_reqs.size;
    mem_info.memoryTypeIndex = FindMemoryType(
        mem_reqs.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    if (!VkOk(vkAllocateMemory(device_, &mem_info, nullptr, &eye.depth_memory),
              "vkAllocateMemory(depth)")) {
      return false;
    }
    vkBindImageMemory(device_, eye.depth_image, eye.depth_memory, 0);

    VkImageViewCreateInfo dv{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
    dv.image = eye.depth_image;
    dv.viewType = VK_IMAGE_VIEW_TYPE_2D;
    dv.format = depth_format_;
    dv.subresourceRange = {VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1};
    if (!VkOk(vkCreateImageView(device_, &dv, nullptr, &eye.depth_view),
              "vkCreateImageView(depth)")) {
      return false;
    }

    eye.color_views.resize(sc.images.size());
    eye.framebuffers.resize(sc.images.size());
    for (size_t img = 0; img < sc.images.size(); ++img) {
      VkImageViewCreateInfo cv{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
      cv.image = sc.images[img].image;
      cv.viewType = VK_IMAGE_VIEW_TYPE_2D;
      cv.format = color_format_;
      cv.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
      if (!VkOk(vkCreateImageView(device_, &cv, nullptr,
                                  &eye.color_views[img]),
                "vkCreateImageView(color)")) {
        return false;
      }
      VkImageView attachments[2] = {eye.color_views[img], eye.depth_view};
      VkFramebufferCreateInfo fb{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
      fb.renderPass = render_pass_;
      fb.attachmentCount = 2;
      fb.pAttachments = attachments;
      fb.width = sc.width;
      fb.height = sc.height;
      fb.layers = 1;
      if (!VkOk(vkCreateFramebuffer(device_, &fb, nullptr,
                                    &eye.framebuffers[img]),
                "vkCreateFramebuffer")) {
        return false;
      }
    }
  }
  return true;
}

VkCommandBuffer VkContext::BeginFrameCommands() {
  vkWaitForFences(device_, 1, &frame_fence_, VK_TRUE, UINT64_MAX);
  vkResetFences(device_, 1, &frame_fence_);
  vkResetCommandBuffer(command_buffer_, 0);
  VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
  begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
  vkBeginCommandBuffer(command_buffer_, &begin);
  return command_buffer_;
}

void VkContext::BeginEyePass(VkCommandBuffer cmd, int eye_index,
                             uint32_t image_index) {
  const EyeTarget& eye = eyes_[eye_index];
  VkClearValue clears[2];
  clears[0].color = {{clear_color_[0], clear_color_[1], clear_color_[2],
                      clear_color_[3]}};
  clears[1].depthStencil = {1.0f, 0};
  VkRenderPassBeginInfo rp{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
  rp.renderPass = render_pass_;
  rp.framebuffer = eye.framebuffers[image_index];
  rp.renderArea = {{0, 0},
                   {static_cast<uint32_t>(eye.width),
                    static_cast<uint32_t>(eye.height)}};
  rp.clearValueCount = 2;
  rp.pClearValues = clears;
  vkCmdBeginRenderPass(cmd, &rp, VK_SUBPASS_CONTENTS_INLINE);

  VkViewport vp{0, 0, static_cast<float>(eye.width),
                static_cast<float>(eye.height), 0.0f, 1.0f};
  vkCmdSetViewport(cmd, 0, 1, &vp);
  VkRect2D scissor{{0, 0},
                   {static_cast<uint32_t>(eye.width),
                    static_cast<uint32_t>(eye.height)}};
  vkCmdSetScissor(cmd, 0, 1, &scissor);
}

void VkContext::EndEyePass(VkCommandBuffer cmd) { vkCmdEndRenderPass(cmd); }

bool VkContext::SubmitAndWait(VkCommandBuffer cmd) {
  vkEndCommandBuffer(cmd);
  VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO};
  submit.commandBufferCount = 1;
  submit.pCommandBuffers = &cmd;
  return VkOk(vkQueueSubmit(queue_, 1, &submit, frame_fence_),
              "vkQueueSubmit");
}

uint32_t VkContext::FindMemoryType(uint32_t type_bits,
                                   VkMemoryPropertyFlags props) {
  VkPhysicalDeviceMemoryProperties mem;
  vkGetPhysicalDeviceMemoryProperties(physical_, &mem);
  for (uint32_t i = 0; i < mem.memoryTypeCount; ++i) {
    if ((type_bits & (1u << i)) &&
        (mem.memoryTypes[i].propertyFlags & props) == props) {
      return i;
    }
  }
  LOGE("no matching memory type (bits 0x%x props 0x%x)", type_bits, props);
  return 0;
}

bool VkContext::CreateBuffer(VkDeviceSize size, VkBufferUsageFlags usage,
                             VkMemoryPropertyFlags props, VkBuffer* buffer,
                             VkDeviceMemory* memory) {
  VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  info.size = size;
  info.usage = usage;
  info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  if (!VkOk(vkCreateBuffer(device_, &info, nullptr, buffer),
            "vkCreateBuffer")) {
    return false;
  }
  VkMemoryRequirements reqs;
  vkGetBufferMemoryRequirements(device_, *buffer, &reqs);
  VkMemoryAllocateInfo alloc{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
  alloc.allocationSize = reqs.size;
  alloc.memoryTypeIndex = FindMemoryType(reqs.memoryTypeBits, props);
  if (!VkOk(vkAllocateMemory(device_, &alloc, nullptr, memory),
            "vkAllocateMemory")) {
    return false;
  }
  vkBindBufferMemory(device_, *buffer, *memory, 0);
  return true;
}

void VkContext::Destroy() {
  if (device_ != VK_NULL_HANDLE) {
    vkDeviceWaitIdle(device_);
    for (auto& eye : eyes_) {
      for (auto fb : eye.framebuffers) {
        vkDestroyFramebuffer(device_, fb, nullptr);
      }
      for (auto view : eye.color_views) {
        vkDestroyImageView(device_, view, nullptr);
      }
      if (eye.depth_view) {
        vkDestroyImageView(device_, eye.depth_view, nullptr);
      }
      if (eye.depth_image) {
        vkDestroyImage(device_, eye.depth_image, nullptr);
      }
      if (eye.depth_memory) {
        vkFreeMemory(device_, eye.depth_memory, nullptr);
      }
    }
    eyes_.clear();
    if (render_pass_) {
      vkDestroyRenderPass(device_, render_pass_, nullptr);
    }
    if (frame_fence_) {
      vkDestroyFence(device_, frame_fence_, nullptr);
    }
    if (command_pool_) {
      vkDestroyCommandPool(device_, command_pool_, nullptr);
    }
    vkDestroyDevice(device_, nullptr);
  }
  if (instance_ != VK_NULL_HANDLE) {
    vkDestroyInstance(instance_, nullptr);
  }
}
