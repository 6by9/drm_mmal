/*
Copyright (c) 2017 Raspberry Pi (Trading) Ltd

Based on example_basic_2.c from the Userland repo (interface/mmal/test/examples)

Amended to use GEM to allocate the video buffers from CMA, export as dmabufs,
import the dmabufs via vcsm, and then use them with MMAL.
Output frames are then presented to libdrm for display.

Plays an H264 elementary stream with no frame scheduling.

Copyright (c) 2012, Broadcom Europe Ltd
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "drm_fourcc.h"
#include <drm.h>
#include <drm_mode.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#include <interface/vcsm/user-vcsm.h>
#include "bcm_host.h"
#include "interface/mmal/mmal.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/mmal_queue.h"
#include "interface/vcos/vcos.h"
#include <stdio.h>

//Tested working:
//MMAL_ENCODING_I420 (YU12)
//MMAL_ENCODING_YV12
//MMAL_ENCODING_NV12
//MMAL_ENCODING_I422 (YU16)
//MMAL_ENCODING_BGRA
//MMAL_ENCODING_RGBA
//MMAL_ENCODING_RGB16 (RGB565)
//
//Valid to the ISP, but not DRM:
//MMAL_ENCODING_NV21 (NV12 with cb/cr swapped)
//MMAL_ENCODING_RGB24
//MMAL_ENCODING_BGR24
//Patches sorted for vc4_plane.c for each of these, and then they work.
//
#define ENCODING_FOR_DRM  MMAL_ENCODING_YUVUV128

#define DRM_MODULE "vc4"
#define MAX_BUFFERS 5

static inline int warn(const char *file, int line, const char *fmt, ...)
{
   int errsv = errno;
   va_list va;
   va_start(va, fmt);
   fprintf(stderr, "WARN(%s:%d): ", file, line);
   vfprintf(stderr, fmt, va);
   va_end(va);
   errno = errsv;
   return 1;
}

#define CHECK_CONDITION(cond, ...) \
do { \
   if (cond) { \
      int errsv = errno; \
      fprintf(stderr, "ERROR(%s:%d) : ", \
         __FILE__, __LINE__); \
      errno = errsv; \
      fprintf(stderr,  __VA_ARGS__); \
      abort(); \
   } \
} while(0)
#define WARN_ON(cond, ...) \
   ((cond) ? warn(__FILE__, __LINE__, __VA_ARGS__) : 0)
#define ERRSTR strerror(errno)
#define CHECK_STATUS(status, ...) WARN_ON(status != MMAL_SUCCESS, __VA_ARGS__); \
   if (status != MMAL_SUCCESS) goto error;

static uint8_t codec_header_bytes[512];
static unsigned int codec_header_bytes_size = sizeof(codec_header_bytes);

static FILE *source_file;

/* Macros abstracting the I/O, just to make the example code clearer */
#define SOURCE_OPEN(uri) \
   source_file = fopen(uri, "rb"); if (!source_file) goto error;
#define SOURCE_READ_CODEC_CONFIG_DATA(bytes, size) \
   size = fread(bytes, 1, size, source_file); rewind(source_file)
#define SOURCE_READ_DATA_INTO_BUFFER(a) \
   a->length = fread(a->data, 1, a->alloc_size - 128, source_file); \
   a->offset = 0
#define SOURCE_CLOSE() \
   if (source_file) fclose(source_file)

/** Context for our application */
static struct CONTEXT_T {
   VCOS_SEMAPHORE_T semaphore;
   MMAL_QUEUE_T *queue;
   MMAL_STATUS_T status;
} context;

struct buffer {
   unsigned int bo_handle;
   unsigned int fb_handle;
   int dbuf_fd;
   unsigned int vcsm_handle;
   MMAL_BUFFER_HEADER_T *mmal_buffer;
};

struct drm_setup {
   int conId;
   uint32_t crtcId;
   int crtcIdx;
   uint32_t planeId;
   unsigned int out_fourcc;
   MMAL_RECT_T compose;
};


static void log_format(MMAL_ES_FORMAT_T *format, MMAL_PORT_T *port)
{
   const char *name_type;

   if(port)
      fprintf(stderr, "%s:%s:%i", port->component->name,
               port->type == MMAL_PORT_TYPE_CONTROL ? "ctr" :
                  port->type == MMAL_PORT_TYPE_INPUT ? "in" :
                  port->type == MMAL_PORT_TYPE_OUTPUT ? "out" : "invalid",
               (int)port->index);

   switch(format->type)
   {
   case MMAL_ES_TYPE_AUDIO: name_type = "audio"; break;
   case MMAL_ES_TYPE_VIDEO: name_type = "video"; break;
   case MMAL_ES_TYPE_SUBPICTURE: name_type = "subpicture"; break;
   default: name_type = "unknown"; break;
   }

   fprintf(stderr, "type: %s, fourcc: %4.4s\n", name_type, (char *)&format->encoding);
   fprintf(stderr, " bitrate: %i, framed: %i\n", format->bitrate,
            !!(format->flags & MMAL_ES_FORMAT_FLAG_FRAMED));
   fprintf(stderr, " extra data: %i, %p\n", format->extradata_size, format->extradata);
   switch(format->type)
   {
   case MMAL_ES_TYPE_AUDIO:
      fprintf(stderr, " samplerate: %i, channels: %i, bps: %i, block align: %i\n",
               format->es->audio.sample_rate, format->es->audio.channels,
               format->es->audio.bits_per_sample, format->es->audio.block_align);
      break;

   case MMAL_ES_TYPE_VIDEO:
      fprintf(stderr, " width: %i, height: %i, (%i,%i,%i,%i)\n",
               format->es->video.width, format->es->video.height,
               format->es->video.crop.x, format->es->video.crop.y,
               format->es->video.crop.width, format->es->video.crop.height);
      fprintf(stderr, " pixel aspect ratio: %i/%i, frame rate: %i/%i\n",
               format->es->video.par.num, format->es->video.par.den,
               format->es->video.frame_rate.num, format->es->video.frame_rate.den);
      break;

   case MMAL_ES_TYPE_SUBPICTURE:
      break;

   default: break;
   }

   if(!port)
      return;

   fprintf(stderr, " buffers num: %i(opt %i, min %i), size: %i(opt %i, min: %i), align: %i\n",
            port->buffer_num, port->buffer_num_recommended, port->buffer_num_min,
            port->buffer_size, port->buffer_size_recommended, port->buffer_size_min,
            port->buffer_alignment_min);
}

uint32_t mmal_encoding_to_drm_fourcc(uint32_t mmal_encoding)
{
   switch(mmal_encoding)
   {
      case MMAL_ENCODING_I420:
         return MMAL_FOURCC('Y','U','1','2');
      case MMAL_ENCODING_YV12:
         return MMAL_FOURCC('Y','V','1','2');
      case MMAL_ENCODING_I422:
         return MMAL_FOURCC('Y','U','1','6');
      case MMAL_ENCODING_YUVUV128:
      case MMAL_ENCODING_NV12:
         return MMAL_FOURCC('N','V','1','2');
      case MMAL_ENCODING_NV21:
         return MMAL_FOURCC('N','V','2','1');
      case MMAL_ENCODING_RGB16:
         return MMAL_FOURCC('R','G','1','6');
      case MMAL_ENCODING_RGB24:
         return MMAL_FOURCC('B','G','2','4');
      case MMAL_ENCODING_BGR24:
         return MMAL_FOURCC('R','G','2','4');
      case MMAL_ENCODING_BGR32:
      case MMAL_ENCODING_BGRA:
         return MMAL_FOURCC('X','R','2','4');
      case MMAL_ENCODING_RGB32:
      case MMAL_ENCODING_RGBA:
         return MMAL_FOURCC('X','B','2','4');
      case MMAL_ENCODING_OPAQUE:
         fprintf(stderr, "MMAL_ENCODING_OPAQUE can't be converted to a DRM compatible format\n");
      default:
         return 0;
   }
}

#define VC_IMAGE_YUV_UV_STRIPE_WIDTH_LOG2 7
#define YUV_UV_MAX_STRIPE_HEIGHT  1264

/* 1920 lines * 128 wide columns is a column stride of 0x3c000 which
 * just fits once aligned with the current scheme (becomes 0x3c800).
 * Adding any more lines than that exceeds the 0x40000 column stride
 * limit required by the codec block.
 */
#define YUV_UV_MAX_TALL_MODE_HEIGHT 1920
#define SAND_ALIGN 4096

static int get_sand_column_pitch(int height)
{
   int pitch;

   if (height <= (YUV_UV_MAX_TALL_MODE_HEIGHT-16))
     height += 16; // for di_adv deinterlace
   /* calculate required space for a Y stripe then pad it to align UV data */
   pitch = ALIGN_UP(ALIGN_UP(height, 16) * 128, SAND_ALIGN);

   /* For tall YUV_UV images, use independent stripes for UV.
    * Pitch copes with only the Y (or the UV); the number of stripes is
    * doubled.
    * (See YUV_UV_MAX_STRIPE_HEIGHT).
    */
   if (height <= YUV_UV_MAX_STRIPE_HEIGHT)
   {
      /* then add the UV stripe space and re-align */
      pitch = ALIGN_UP(pitch * 3 >> 1, SAND_ALIGN);
   }

   /*
   The requirement in the source that the stride be an odd multiple of two pages in size is actually over-restrictive.
   The intention is that in a four-bank SDRAM if a motion compensation source location overlaps a stripe boundary in X
   and a page transition in Y then the four pages covered are in different banks (call this the "four corners property").
   As we have an eight-bank device we get the desired effect if the stride is a multiple of the page size and stride/pagesize & 7 != 0, 1 or 7.
   On Pi 2 our wacky custom SDRAM gives us 8k (64-line) pages, while on Pi 1 we have 4k (32-line) pages.
   I think our best bet to satisfy the "four corners" property and avoid aliasing is to pad stripes to an odd multiple of 2k (an odd multiple of 16 lines),
   and to avoid multiples which bring the banks in adjacent stripes so nearly into alignment that a 23 pixel high motcomp neighbourhood
   can reach from a bank n page in stripe x to a bank n page in stripe x+1.
   */
   int h;
   for (h = pitch >> (VC_IMAGE_YUV_UV_STRIPE_WIDTH_LOG2 + 4); ; h++)
   {
     if (0) //vc_image_helper_get_sdram_page_size() == 8192)
     {
       int hh = h % 32;
       if ((hh & 1) && hh >= 7 && hh <= 25)
         break;
     }
     else
     {
       //vcos_assert(vc_image_helper_get_sdram_page_size() == 4096);
       int hh = h % 16;
       if ((hh & 1) && hh >= 5 && hh <= 11)
         break;
     }
   }
   h <<= 4;
   return h;
}

void mmal_format_to_drm_pitches_offsets(uint32_t *pitches, uint32_t *offsets,
            uint64_t *modifiers, uint32_t *bo_handles, MMAL_ES_FORMAT_T *format)
{
   switch (format->encoding)
   {
      // 3 plane YUV formats
      case MMAL_ENCODING_I420:
      case MMAL_ENCODING_YV12:
         pitches[0] = mmal_encoding_width_to_stride(format->encoding, format->es->video.width);
         pitches[1] = pitches[0] / 2;
         pitches[2] = pitches[1];

         offsets[1] = pitches[0] * format->es->video.height;
         offsets[2] = offsets[1] + pitches[1] * format->es->video.height/2;

         bo_handles[1] = bo_handles[2] = bo_handles[0];
         break;
      case MMAL_ENCODING_I422:
         pitches[0] = mmal_encoding_width_to_stride(format->encoding, format->es->video.width);
         pitches[1] = pitches[0] / 2;
         pitches[2] = pitches[1];

         offsets[1] = pitches[0] * format->es->video.height;
         offsets[2] = offsets[1] + pitches[1] * format->es->video.height;

         bo_handles[1] = bo_handles[2] = bo_handles[0];
         break;
      // 2 plane YUV formats
      case MMAL_ENCODING_NV12:
      case MMAL_ENCODING_NV21:
         pitches[0] = mmal_encoding_width_to_stride(format->encoding, format->es->video.width);
         pitches[1] = pitches[0];

         offsets[1] = pitches[0] * format->es->video.height;

         bo_handles[1] = bo_handles[0];
         break;
      case MMAL_ENCODING_YUVUV128:
         pitches[0] = format->es->video.width;    //Should be 128, but DRM rejects that.
         modifiers[0] = DRM_FORMAT_MOD_BROADCOM_SAND128_COL_HEIGHT(get_sand_column_pitch(format->es->video.height));
         printf("Modifier set as %llu, param %u\n", modifiers[0], fourcc_mod_broadcom_param(modifiers[0]));
         modifiers[1] = modifiers[0];
         pitches[1] = pitches[0];

         offsets[1] = 128 * (format->es->video.height+32);  //ISP seems to produce an extra 16 lines of padding. Don't know why.

         bo_handles[1] = bo_handles[0];
         break;
      default:
         pitches[0] = mmal_encoding_width_to_stride(format->encoding, format->es->video.width);
         break;
   }
   printf("MMAL format %08x translates to pitches [%u,%u,%u,%u], and offsets [%u,%u,%u,%u]\n",
            format->encoding,
            pitches[0],pitches[1],pitches[2],pitches[3],
            offsets[0],offsets[1],offsets[2],offsets[3]
            );
}

#define BUFFER_IMPORT 0
static int buffer_create(struct buffer *b, int drmfd, MMAL_PORT_T *port)
{
   uint32_t offsets[4] = { 0 };
   uint32_t pitches[4] = { 0 };
   uint32_t bo_handles[4] = { 0 };
   uint64_t modifiers[4] = { 0 };
   unsigned int fourcc = mmal_encoding_to_drm_fourcc(port->format->encoding);
   int ret;

#if BUFFER_IMPORT
   struct drm_mode_create_dumb gem;
   struct drm_mode_destroy_dumb gem_destroy;

   memset(&gem, 0, sizeof gem);
   gem.width = port->format->es->video.width;
   gem.height = port->format->es->video.height;
   gem.bpp = 32;
   gem.size = port->buffer_size;
   ret = ioctl(drmfd, DRM_IOCTL_MODE_CREATE_DUMB, &gem);
   if (ret)
   {
      printf("CREATE_DUMB failed: %s\n", ERRSTR);
      return -1;
   }
   printf("bo %u %ux%u bpp %u size %lu (%u)\n", gem.handle, gem.width, gem.height, gem.bpp, (long)gem.size, port->buffer_size);
   b->bo_handle = gem.handle;

   struct drm_prime_handle prime;
   memset(&prime, 0, sizeof prime);
   prime.handle = b->bo_handle;

   ret = ioctl(drmfd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &prime);
   if (ret)
   {
      printf("PRIME_HANDLE_TO_FD failed: %s\n", ERRSTR);
      goto fail_gem;
   }
   printf("dbuf_fd = %d\n", prime.fd);
   b->dbuf_fd = prime.fd;

   b->vcsm_handle = vcsm_import_dmabuf(b->dbuf_fd, "DRM Buf");
   if (!b->vcsm_handle)
      goto fail_prime;
#else
   b->vcsm_handle = vcsm_malloc(port->buffer_size, "DRM Buf");
   if (!b->vcsm_handle)
   {
      printf("vcsm_malloc failed: %s\n", ERRSTR);
      return -1;
   }

   b->dbuf_fd = vcsm_export_dmabuf(b->vcsm_handle);
   if (b->dbuf_fd < 0)
   {
      printf("vcsm_export failed: %s\n", ERRSTR);
      goto fail_vcsm;
   }

   //struct drm_prime_handle prime;
   //memset(&prime, 0, sizeof prime);
   //prime.fd = b->dbuf_fd;

   //ret = ioctl(drmfd, DRM_IOCTL_PRIME_FD_TO_HANDLE, &prime);
   ret = drmPrimeFDToHandle(drmfd, b->dbuf_fd, &b->bo_handle);
   if (ret)
   {
      printf("DRM_IOCTL_PRIME_FD_TO_HANDLE failed: %s\n", ERRSTR);
      goto fail_unexport;
   }
   //b->bo_handle = prime.handle;
   printf("prime_bo = %d\n", b->bo_handle);


#endif

   bo_handles[0] = b->bo_handle;

   mmal_format_to_drm_pitches_offsets(pitches, offsets, modifiers, bo_handles, port->format);

   fprintf(stderr, "FB fourcc %c%c%c%c\n",
      fourcc,
      fourcc >> 8,
      fourcc >> 16,
      fourcc >> 24);

   ret = drmModeAddFB2WithModifiers(drmfd, port->format->es->video.crop.width,
//   ret = drmModeAddFB2(drmfd, port->format->es->video.crop.width,
      port->format->es->video.crop.height, fourcc, bo_handles,
      pitches, offsets, modifiers, &b->fb_handle, modifiers[0] ? DRM_MODE_FB_MODIFIERS : 0);
//      pitches, offsets, &b->fb_handle, 0);
   if (ret)
   {
      printf("drmModeAddFB2 failed: %s\n", ERRSTR);
      goto fail_all;
   }

   return 0;

fail_all:
#if BUFFER_IMPORT
   vcsm_free(b->vcsm_handle);
fail_prime:
   close(b->dbuf_fd);

fail_gem:
   memset(&gem_destroy, 0, sizeof gem_destroy);
   gem_destroy.handle = b->bo_handle,
   ret = ioctl(drmfd, DRM_IOCTL_MODE_DESTROY_DUMB, &gem_destroy);
   if (ret)
   {
      printf("DESTROY_DUMB failed: %s\n", ERRSTR);
   }
#else
fail_free_all:
   {
      //How do I release the PRIME_FD_TO_HANDLE handle? Is it this?
      struct drm_gem_close gem_close;

      memset(&gem_close, 0, sizeof gem_close);
      gem_close.handle = b->bo_handle;
      if (drmIoctl(drmfd, DRM_IOCTL_GEM_CLOSE, &gem_close) < 0)
         printf("cant close gem: %s\n", strerror(errno));
   }
fail_unexport:
   close(b->dbuf_fd);
fail_vcsm:
   vcsm_free(b->vcsm_handle);
#endif

   return -1;
}

MMAL_POOL_T* pool_create_drm(MMAL_PORT_T *port, struct buffer *buffers, int drmfd)
{
   MMAL_POOL_T *pool;
   unsigned int i;

   pool = mmal_port_pool_create(port,
                                port->buffer_num,
                                0);

   for (i = 0; i < port->buffer_num; i++)
   {
      buffer_create(&buffers[i], drmfd, port);
      
      pool->header[i]->data = (uint8_t*)vcsm_vc_hdl_from_hdl(buffers[i].vcsm_handle);
      pool->header[i]->alloc_size = port->buffer_size;
      pool->header[i]->length = 0;
      buffers[i].mmal_buffer = pool->header[i];
   }

   return pool;
}

void buffer_destroy(int drmfd, struct buffer *buf)
{
#if BUFFER_IMPORT
   struct drm_mode_destroy_dumb gem_destroy;

   vcsm_free(buf->vcsm_handle);

   close(buf->dbuf_fd);

   memset(&gem_destroy, 0, sizeof gem_destroy);
   gem_destroy.handle = buf->bo_handle;
   ioctl(drmfd, DRM_IOCTL_MODE_DESTROY_DUMB, &gem_destroy);
#else
   //How do I release the PRIME_FD_TO_HANDLE handle? Is it this?
   {
      struct drm_gem_close gem_close;

      memset(&gem_close, 0, sizeof gem_close);
      gem_close.handle = buf->bo_handle;
      if (drmIoctl(drmfd, DRM_IOCTL_GEM_CLOSE, &gem_close) < 0)
         printf("cant close gem: %s\n", strerror(errno));
   }
   
   close(buf->dbuf_fd);
   vcsm_free(buf->vcsm_handle);
#endif
}

void pool_destroy_drm(MMAL_PORT_T *port, MMAL_POOL_T *pool, struct buffer *buffers, int drmfd)
{
   unsigned int i;

   for (i = 0; i < pool->headers_num; i++)
   {
      buffer_destroy(drmfd, &buffers[i]);
   }
   mmal_port_pool_destroy(port, pool);
}

/** Callback from the control port.
 * Component is sending us an event. */
static void control_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
   struct CONTEXT_T *ctx = (struct CONTEXT_T *)port->userdata;

   switch (buffer->cmd)
   {
   case MMAL_EVENT_EOS:
      /* Only sink component generate EOS events */
      break;
   case MMAL_EVENT_ERROR:
      /* Something went wrong. Signal this to the application */
      ctx->status = *(MMAL_STATUS_T *)buffer->data;
      break;
   default:
      break;
   }

   /* Done with the event, recycle it */
   mmal_buffer_header_release(buffer);

   /* Kick the processing thread */
   vcos_semaphore_post(&ctx->semaphore);
}

static int find_crtc(int drmfd, struct drm_setup *s, uint32_t *con)
{
   int ret = -1;
   int i;
   drmModeRes *res = drmModeGetResources(drmfd);
   if(!res) 
   {
      printf( "drmModeGetResources failed: %s\n", ERRSTR);
      return -1;
   }

   if (res->count_crtcs <= 0)
   {
      printf( "drm: no crts\n");
      goto fail_res;
   }

   if (!s->conId) {
      fprintf(stderr,
         "No connector ID specified.  Choosing default from list:\n");

      for (i = 0; i < res->count_connectors; i++) {
         drmModeConnector *con =
            drmModeGetConnector(drmfd, res->connectors[i]);
         drmModeEncoder *enc = NULL;
         drmModeCrtc *crtc = NULL;

         if (con->encoder_id) {
            enc = drmModeGetEncoder(drmfd, con->encoder_id);
            if (enc->crtc_id) {
               crtc = drmModeGetCrtc(drmfd, enc->crtc_id);
            }
         }

         if (!s->conId && crtc) {
            s->conId = con->connector_id;
            s->crtcId = crtc->crtc_id;
         }

         printf("Connector %d (crtc %d): type %d, %dx%d%s\n",
                con->connector_id,
                crtc ? crtc->crtc_id : 0,
                con->connector_type,
                crtc ? crtc->width : 0,
                crtc ? crtc->height : 0,
                (s->conId == (int)con->connector_id ?
            " (chosen)" : ""));
      }

      if (!s->conId) {
         fprintf(stderr,
            "No suitable enabled connector found.\n");
         exit(1);
      }
   }

   s->crtcIdx = -1;

   for (i = 0; i < res->count_crtcs; ++i) {
      if (s->crtcId == res->crtcs[i]) {
         s->crtcIdx = i;
         break;
      }
   }

   if (WARN_ON(s->crtcIdx == -1, "drm: CRTC %u not found\n", s->crtcId))
      goto fail_res;

   if (WARN_ON(res->count_connectors <= 0, "drm: no connectors\n"))
      goto fail_res;

   drmModeConnector *c;
   c = drmModeGetConnector(drmfd, s->conId);
   if (WARN_ON(!c, "drmModeGetConnector failed: %s\n", ERRSTR))
      goto fail_res;

   if (WARN_ON(!c->count_modes, "connector supports no mode\n"))
      goto fail_conn;

   {
      drmModeCrtc *crtc = drmModeGetCrtc(drmfd, s->crtcId);
      s->compose.x = crtc->x;
      s->compose.y = crtc->y;
      s->compose.width = crtc->width;
      s->compose.height = crtc->height;
      drmModeFreeCrtc(crtc);
   }

   if (con)
      *con = c->connector_id;
   ret = 0;

fail_conn:
   drmModeFreeConnector(c);

fail_res:
   drmModeFreeResources(res);

   return ret;
}

static int find_plane(int drmfd, struct drm_setup *s)
{
   drmModePlaneResPtr planes;
   drmModePlanePtr plane;
   unsigned int i;
   unsigned int j;
   int ret = 0;

   planes = drmModeGetPlaneResources(drmfd);
   if (WARN_ON(!planes, "drmModeGetPlaneResources failed: %s\n", ERRSTR))
      return -1;

   for (i = 0; i < planes->count_planes; ++i) {
      plane = drmModeGetPlane(drmfd, planes->planes[i]);
      if (WARN_ON(!planes, "drmModeGetPlane failed: %s\n", ERRSTR))
         break;

      if (!(plane->possible_crtcs & (1 << s->crtcIdx))) {
         drmModeFreePlane(plane);
         continue;
      }

      for (j = 0; j < plane->count_formats; ++j) {
         if (plane->formats[j] == s->out_fourcc)
            break;
      }

      if (j == plane->count_formats) {
         drmModeFreePlane(plane);
         continue;
      }

      s->planeId = plane->plane_id;
      drmModeFreePlane(plane);
      break;
   }

   if (i == planes->count_planes)
      ret = -1;

   drmModeFreePlaneResources(planes);
   return ret;
}


/** Callback from the input port.
 * Buffer has been consumed and is available to be used again. */
static void input_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
   struct CONTEXT_T *ctx = (struct CONTEXT_T *)port->userdata;

   /* The decoder is done with the data, just recycle the buffer header into its pool */
   mmal_buffer_header_release(buffer);

   /* Kick the processing thread */
   vcos_semaphore_post(&ctx->semaphore);
}

/** Callback from the output port.
 * Buffer has been produced by the port and is available for processing. */
static void output_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
   struct CONTEXT_T *ctx = (struct CONTEXT_T *)port->userdata;
   /* Queue the decoded video frame */
   //fprintf(stderr, "Buffer %p returned cmd %08X, length %u\n", buffer, buffer->cmd, buffer->length);
   mmal_queue_put(ctx->queue, buffer);

   /* Kick the processing thread */
   vcos_semaphore_post(&ctx->semaphore);
}

static void drm_mmal_connection_cb(MMAL_CONNECTION_T *connection)
{
   struct CONTEXT_T *ctx = (struct CONTEXT_T *)connection->user_data;
   vcos_semaphore_post(&ctx->semaphore);
}

static int drm_mmal_create_buffers(MMAL_PORT_T *port, struct buffer *buffers, int drmfd, MMAL_POOL_T **pool_out)
{
   MMAL_STATUS_T status;
   port->buffer_size = port->buffer_size_min;
   printf("Set buffer size to %d\n", port->buffer_size);
   status = mmal_port_format_commit(port);

   *pool_out = pool_create_drm(port,
                              buffers, drmfd);

   status = mmal_port_enable(port, output_callback);
   CHECK_STATUS(status, "failed to enable port");
   return 0;
error:
   return status;
}

static int drm_mmal_destroy_buffers(MMAL_PORT_T *port, struct buffer *buffers, int drmfd, MMAL_POOL_T *pool_out)
{
   MMAL_STATUS_T status;

   status = mmal_port_disable(port);
   CHECK_STATUS(status, "failed to disable port");

   //Clear the queue of all buffers
   while(mmal_queue_length(pool_out->queue) != pool_out->headers_num)
   {
      MMAL_BUFFER_HEADER_T *buf;
      fprintf(stderr, "Wait for buffers to be returned. Have %d of %d buffers\n",
            mmal_queue_length(pool_out->queue), pool_out->headers_num);
      vcos_semaphore_wait(&context.semaphore);
      fprintf(stderr, "Got semaphore\n");
      buf = mmal_queue_get(context.queue);
      printf("Retrieved buf %p\n", buf);
      if (buf)
         mmal_buffer_header_release(buf);
   }
   fprintf(stderr, "Got all buffers\n");

   pool_destroy_drm(port, pool_out, buffers, drmfd);
   return 0;
error:
   return status;
}

int main(int argc, char **argv)
{
   MMAL_STATUS_T status = MMAL_EINVAL;
   MMAL_COMPONENT_T *decoder = NULL, *isp = NULL;
   MMAL_POOL_T *pool_in = NULL, *pool_out = NULL;
   MMAL_BOOL_T eos_sent = MMAL_FALSE, eos_received = MMAL_FALSE;
   struct drm_setup setup = {0};
   struct buffer buffers[MAX_BUFFERS];
   MMAL_BUFFER_HEADER_T *current_buffer = NULL, *previous_buffer = NULL, *prev_prev_buffer = NULL;
   MMAL_CONNECTION_T *connection = NULL;
   unsigned int in_count = 0, conn_out_count = 0, conn_in_count = 0, out_count = 0;
   int ret;

   if (argc < 2)
   {
      fprintf(stderr, "invalid arguments\n");
      return -1;
   }

   bcm_host_init();

   int drmfd = drmOpen(DRM_MODULE, NULL);
   CHECK_CONDITION(drmfd < 0, "drmOpen(%s) failed: %s\n", DRM_MODULE, ERRSTR);

   vcsm_init_ex(1, -1);

   vcos_semaphore_create(&context.semaphore, "example", 1);

   SOURCE_OPEN(argv[1]);

   /* Create the decoder component.
    * This specific component exposes 2 ports (1 input and 1 output). Like most components
    * its expects the format of its input port to be set by the client in order for it to
    * know what kind of data it will be fed. */
   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_DECODER, &decoder);
   CHECK_STATUS(status, "failed to create decoder");

   status = mmal_component_create("vc.ril.isp", &isp);
   CHECK_STATUS(status, "failed to create isp");

   /* Enable control port so we can receive events from the component */
   decoder->control->userdata = (void *)&context;
   status = mmal_port_enable(decoder->control, control_callback);
   CHECK_STATUS(status, "failed to enable control port");

   /* Set the zero-copy parameter on the input port */
   status = mmal_port_parameter_set_boolean(decoder->input[0], MMAL_PARAMETER_ZERO_COPY, MMAL_FALSE);
   CHECK_STATUS(status, "failed to set zero copy - %s", decoder->input[0]->name);

   /* Set the zero-copy parameter on the output port */
   status = mmal_port_parameter_set_boolean(decoder->output[0], MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
   CHECK_STATUS(status, "failed to set zero copy - %s", decoder->output[0]->name);

   status = mmal_port_parameter_set_boolean(isp->input[0], MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
   CHECK_STATUS(status, "failed to set zero copy - %s", isp->input[0]->name);

   status = mmal_port_parameter_set_boolean(isp->output[0], MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
   CHECK_STATUS(status, "failed to set zero copy - %s", isp->output[0]->name);

   /* Set format of video decoder input port */
   MMAL_ES_FORMAT_T *format_in = decoder->input[0]->format;
   format_in->type = MMAL_ES_TYPE_VIDEO;
   format_in->encoding = MMAL_ENCODING_H264;
   format_in->es->video.width = 1280;
   format_in->es->video.height = 720;
   format_in->es->video.frame_rate.num = 30;
   format_in->es->video.frame_rate.den = 1;
   format_in->es->video.par.num = 1;
   format_in->es->video.par.den = 1;
   /* If the data is known to be framed then the following flag should be set:
    * format_in->flags |= MMAL_ES_FORMAT_FLAG_FRAMED; */

   SOURCE_READ_CODEC_CONFIG_DATA(codec_header_bytes, codec_header_bytes_size);
   status = mmal_format_extradata_alloc(format_in, codec_header_bytes_size);
   CHECK_STATUS(status, "failed to allocate extradata");
   format_in->extradata_size = codec_header_bytes_size;
   if (format_in->extradata_size)
      memcpy(format_in->extradata, codec_header_bytes, format_in->extradata_size);

   status = mmal_port_format_commit(decoder->input[0]);
   CHECK_STATUS(status, "failed to commit format");

   MMAL_ES_FORMAT_T *format_out = decoder->output[0]->format;
   format_out->encoding = MMAL_ENCODING_OPAQUE;

   status = mmal_port_format_commit(decoder->output[0]);
   CHECK_STATUS(status, "failed to commit format");

   status = mmal_connection_create(&connection, decoder->output[0], isp->input[0], 0);
   CHECK_STATUS(status, "failed to create connection");

   (*connection).callback = drm_mmal_connection_cb;
   (*connection).user_data = (void *)&context;

   /* Display the output port format */
   fprintf(stderr, "%s\n", decoder->output[0]->name);
   fprintf(stderr, " type: %i, fourcc: %4.4s\n", format_out->type, (char *)&format_out->encoding);
   fprintf(stderr, " bitrate: %i, framed: %i\n", format_out->bitrate,
           !!(format_out->flags & MMAL_ES_FORMAT_FLAG_FRAMED));
   fprintf(stderr, " extra data: %i, %p\n", format_out->extradata_size, format_out->extradata);
   fprintf(stderr, " width: %i, height: %i, (%i,%i,%i,%i)\n",
           format_out->es->video.width, format_out->es->video.height,
           format_out->es->video.crop.x, format_out->es->video.crop.y,
           format_out->es->video.crop.width, format_out->es->video.crop.height);

   /* The format of both ports is now set so we can get their buffer requirements and create
    * our buffer headers. We use the buffer pool API to create these. */
   decoder->input[0]->buffer_num = decoder->input[0]->buffer_num_min;
   decoder->input[0]->buffer_size = decoder->input[0]->buffer_size_min;
   decoder->output[0]->buffer_num = decoder->output[0]->buffer_num_recommended;
   decoder->output[0]->buffer_size = decoder->output[0]->buffer_size_min;
   pool_in = mmal_port_pool_create(decoder->input[0],
                                   decoder->input[0]->buffer_num,
                                   decoder->input[0]->buffer_size);

   /* Create a queue to store our decoded video frames. The callback we will get when
    * a frame has been decoded will put the frame into this queue. */
   context.queue = mmal_queue_create();

   /* Store a reference to our context in each port (will be used during callbacks) */
   decoder->input[0]->userdata = (void *)&context;

   /* Enable all the input port and the output port.
    * The callback specified here is the function which will be called when the buffer header
    * we sent to the component has been processed. */
   status = mmal_port_enable(decoder->input[0], input_callback);
   CHECK_STATUS(status, "failed to enable input port");
   status = mmal_connection_enable(connection);
   CHECK_STATUS(status, "failed to enable connection");

   /* Component won't start processing data until it is enabled. */
   status = mmal_component_enable(decoder);
   CHECK_STATUS(status, "failed to enable decoder component");

   status = mmal_component_enable(isp);
   CHECK_STATUS(status, "failed to enable isp component");

   isp->output[0]->userdata = (void *)&context;
   mmal_format_full_copy(isp->output[0]->format, isp->input[0]->format);
   isp->output[0]->format->encoding = ENCODING_FOR_DRM;

   status = mmal_port_format_commit(isp->output[0]);
   CHECK_STATUS(status, "failed to set ISP output format");
   isp->output[0]->buffer_num = MAX_BUFFERS;
   isp->output[0]->buffer_size = isp->output[0]->buffer_size_min;

   status = mmal_port_enable(isp->output[0], output_callback);
   CHECK_STATUS(status, "failed to enable isp output port");

   pool_out = pool_create_drm(isp->output[0], buffers, drmfd);
   setup.out_fourcc = mmal_encoding_to_drm_fourcc(ENCODING_FOR_DRM);

   uint32_t con;
   ret = find_crtc(drmfd, &setup, &con);
   CHECK_CONDITION(ret, "failed to find valid mode\n");

   ret = find_plane(drmfd, &setup);
   CHECK_CONDITION(ret, "failed to find compatible plane\n");


   /* Start decoding */
   fprintf(stderr, "start decoding\n");

   /* This is the main processing loop */
   while(!eos_received && out_count < 10000)
   {
      MMAL_BUFFER_HEADER_T *buffer;
      VCOS_STATUS_T vcos_status;

      /* Wait for buffer headers to be available on either of the decoder ports */
      vcos_status = vcos_semaphore_wait_timeout(&context.semaphore, 2000);
      if (vcos_status != VCOS_SUCCESS)
         fprintf(stderr, "vcos_semaphore_wait_timeout failed - status %d\n", vcos_status);

      /* Check for errors */
      if (context.status != MMAL_SUCCESS)
         break;

      /* Send data to decode to the input port of the video decoder */
      if (!eos_sent && (buffer = mmal_queue_get(pool_in->queue)) != NULL)
      {
         SOURCE_READ_DATA_INTO_BUFFER(buffer);
         if(!buffer->length) eos_sent = MMAL_TRUE;

         buffer->flags = buffer->length ? 0 : MMAL_BUFFER_HEADER_FLAG_EOS;
         buffer->pts = buffer->dts = MMAL_TIME_UNKNOWN;
         //fprintf(stderr, "sending %i bytes\n", (int)buffer->length);
         status = mmal_port_send_buffer(decoder->input[0], buffer);
         CHECK_STATUS(status, "failed to send buffer");
         in_count++;
         //fprintf(stderr, "Input buffer %p to port %s. in_count %u\n", buffer, decoder->input[0]->name, in_count);
      }

      /* Get buffers from the connection */
      while ((buffer = mmal_queue_get(connection->queue)) != NULL)
      {
         conn_out_count++;
         //fprintf(stderr, "Connection buffer %p to port %s. conn_out_count %u\n", buffer, connection->in->name, conn_out_count);
         if (buffer->cmd)
         {
            fprintf(stderr, "received event %4.4s\n", (char *)&buffer->cmd);
            if (buffer->cmd == MMAL_EVENT_FORMAT_CHANGED)
            {
               MMAL_EVENT_FORMAT_CHANGED_T *event = mmal_event_format_changed_get(buffer);
               if (event)
               {
                  fprintf(stderr, "----------Port format changed----------\n");
                  log_format(decoder->output[0]->format, decoder->output[0]);
                  fprintf(stderr, "-----------------to---------------------\n");
                  log_format(event->format, 0);
                  fprintf(stderr, " buffers num (opt %i, min %i), size (opt %i, min: %i)\n",
                           event->buffer_num_recommended, event->buffer_num_min,
                           event->buffer_size_recommended, event->buffer_size_min);
                  fprintf(stderr, "----------------------------------------\n");
               }

               status = mmal_connection_event_format_changed(connection, buffer);
               //Assume we can't reuse the output buffers, so have to disable, destroy
               //pool, create new pool, enable port, feed in buffers.
               status = drm_mmal_destroy_buffers(isp->output[0], buffers, drmfd, pool_out);

               status = mmal_format_full_copy(isp->output[0]->format, event->format);
               isp->output[0]->format->encoding = ENCODING_FOR_DRM;
               isp->output[0]->buffer_num = MAX_BUFFERS;
               isp->output[0]->buffer_size = isp->output[0]->buffer_size_min;

               if (status == MMAL_SUCCESS)
                  status = mmal_port_format_commit(isp->output[0]);
               if (status != MMAL_SUCCESS)
               {
                  fprintf(stderr, "commit failed on output - %d\n", status);
               }

               status = drm_mmal_create_buffers(isp->output[0], buffers, drmfd, &pool_out);
            }
            mmal_buffer_header_release(buffer);
            continue;
         }

         status = mmal_port_send_buffer(connection->in, buffer);
         if (status != MMAL_SUCCESS)
         {
            fprintf(stderr, "mmal_port_send_buffer failed (%i)\n", status);
            mmal_queue_put_back(connection->queue, buffer);
            goto error;
         }
         buffer = mmal_queue_get(connection->queue);
      }

      /* Send empty buffers to the output port of the connection */
      if (connection->pool)
      {
         while ((buffer = mmal_queue_get(connection->pool->queue)) != NULL)
         {
            //fprintf(stderr, "Returning buffer %p to port %s. conn_in_count %u\n", buffer, connection->out->name, conn_in_count);

            status = mmal_port_send_buffer(connection->out, buffer);
            if (status != MMAL_SUCCESS)
            {
               fprintf(stderr, "mmal_port_send_buffer failed (%i)\n", status);
               mmal_queue_put_back(connection->pool->queue, buffer);
               goto error;
            }
            buffer = mmal_queue_get(connection->pool->queue);
            conn_in_count++;
         }
      }

      /* Get our output frames */
      while ((buffer = mmal_queue_get(context.queue)) != NULL)
      {
         /* We have a frame, do something with it (why not display it for instance?).
          * Once we're done with it, we release it. It will automatically go back
          * to its original pool so it can be reused for a new video frame.
          */
         eos_received = buffer->flags & MMAL_BUFFER_HEADER_FLAG_EOS;

         if (buffer->cmd)
         {
            fprintf(stderr, "received event %4.4s\n", (char *)&buffer->cmd);
            if (buffer->cmd == MMAL_EVENT_FORMAT_CHANGED)
            {
               fprintf(stderr, "Unexpected FORMAT_CHANGED event on the output of ISP\n");
            }
            mmal_buffer_header_release(buffer);
         }
         else
         {
            int index;

            //fprintf(stderr, "decoded frame (flags %x) count %d\n", buffer->flags, out_count);
            if (buffer)
            {
               unsigned int i;
               for (i = 0; i < pool_out->headers_num; i++) {
                  if (buffers[i].mmal_buffer == buffer) {
                     //printf("Matches buffer index %u\n", i);
                     index = i;
                     break;
                  }
               }
               if (i == pool_out->headers_num) {
                  printf("Failed to find matching buffer for mmal buffer %p\n", buffer);
                  continue;
               }
            }
            else
               continue;

            ret = drmModeSetPlane(drmfd, setup.planeId, setup.crtcId,
                        buffers[index].fb_handle, 0,
                        setup.compose.x, setup.compose.y,
                        setup.compose.width,
                        setup.compose.height,
                        0, 0,
                        isp->output[0]->format->es->video.crop.width << 16,
                        isp->output[0]->format->es->video.crop.height << 16);
            CHECK_CONDITION(ret, "drmModeSetPlane failed: %s\n", ERRSTR);

            //Release buffer that was on the screen
            if (prev_prev_buffer)
               mmal_buffer_header_release(prev_prev_buffer);
            if (previous_buffer)
               prev_prev_buffer = previous_buffer;
            if (current_buffer)
               previous_buffer = current_buffer;

            //Store pointer to the new buffer that is now on the screen
            current_buffer = buffer;
            out_count++;
         }
      }

      /* Send empty buffers to the output port of the decoder */
      while ((buffer = mmal_queue_get(pool_out->queue)) != NULL)
      {
         //printf("Sending buf %p\n", buffer);
         status = mmal_port_send_buffer(isp->output[0], buffer);
         CHECK_STATUS(status, "failed to send buffer to isp");
      }
   }

   /* Stop decoding */
   fprintf(stderr, "stop decoding\n");

   /* Stop everything. Not strictly necessary since mmal_component_destroy()
    * will do that anyway */
   mmal_port_disable(decoder->input[0]);
   mmal_port_disable(decoder->output[0]);
   mmal_component_disable(decoder);

 error:
   /* Cleanup everything */
   if (pool_in)
      mmal_port_pool_destroy(decoder->input[0], pool_in);
   if (connection)
   {
      mmal_connection_disable(connection);
      mmal_connection_destroy(connection);
   }
   if (pool_out)
      pool_destroy_drm(isp->output[0], pool_out, buffers, drmfd);
   if (decoder)
      mmal_component_destroy(decoder);
   if (isp)
      mmal_component_destroy(isp);
   if (context.queue)
      mmal_queue_destroy(context.queue);

   SOURCE_CLOSE();
   vcos_semaphore_delete(&context.semaphore);
   return status == MMAL_SUCCESS ? 0 : -1;
}
