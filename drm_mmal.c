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
/* Portions of the code lifted from glxgears, under: */
/*
 * Copyright (C) 1999-2001  Brian Paul   All Rights Reserved.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * BRIAN PAUL BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
 * AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* Portions of the code lifted from piglit, under: */
/*
 * Copyright (c) The Piglit project 2007
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * on the rights to use, copy, modify, merge, publish, distribute, sub
 * license, and/or sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.  IN NO EVENT SHALL
 * VA LINUX SYSTEM, IBM AND/OR THEIR SUPPLIERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
 * USE OR OTHER DEALINGS IN THE SOFTWARE.
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

#include <drm.h>
#include <drm_mode.h>
#include <drm_fourcc.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xlib-xcb.h>
#include <epoxy/gl.h>
#include <epoxy/egl.h>
#include <xcb/xcb.h>
#include <xcb/dri3.h>

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
#define ENCODING_FOR_DRM  MMAL_ENCODING_RGBA

#define DRM_MODULE "vc4"
#define MAX_BUFFERS 3

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
   int dbuf_fd;
   unsigned int vcsm_handle;
   MMAL_BUFFER_HEADER_T *mmal_buffer;
   GLuint texture;
};

struct drm_setup {
   Display *dpy;
   EGLDisplay egl_dpy;
   EGLContext ctx;
   EGLSurface surf;
   Window win;
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

void mmal_format_to_drm_pitches_offsets(uint32_t *pitches, uint32_t *offsets, uint32_t *bo_handles, MMAL_ES_FORMAT_T *format)
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

static int buffer_create(struct buffer *b, int drmfd, MMAL_PORT_T *port,
                         EGLDisplay dpy, EGLContext ctx)
{
   struct drm_mode_create_dumb gem;
   struct drm_mode_destroy_dumb gem_destroy;
   int ret;

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

   uint32_t pitches[4] = { 0 };
   uint32_t offsets[4] = { 0 };
   uint32_t bo_handles[4] = { b->bo_handle };
   unsigned int fourcc = mmal_encoding_to_drm_fourcc(port->format->encoding);

   mmal_format_to_drm_pitches_offsets(pitches, offsets, bo_handles, port->format);

   
   fprintf(stderr, "FB fourcc %c%c%c%c\n",
      fourcc,
      fourcc >> 8,
      fourcc >> 16,
      fourcc >> 24);

   b->vcsm_handle = vcsm_import_dmabuf(b->dbuf_fd, "DRM Buf");
   if (!b->vcsm_handle)
      goto fail_prime;

   EGLint attribs[50];
   int i = 0;

   attribs[i++] = EGL_WIDTH;
   attribs[i++] = port->format->es->video.width;
   attribs[i++] = EGL_HEIGHT;
   attribs[i++] = port->format->es->video.height;

   attribs[i++] = EGL_LINUX_DRM_FOURCC_EXT;
   attribs[i++] = fourcc;

   attribs[i++] = EGL_DMA_BUF_PLANE0_FD_EXT;
   attribs[i++] = b->dbuf_fd;

   attribs[i++] = EGL_DMA_BUF_PLANE0_OFFSET_EXT;
   attribs[i++] = offsets[0];

   attribs[i++] = EGL_DMA_BUF_PLANE0_PITCH_EXT;
   attribs[i++] = pitches[0];

   if (pitches[1]) {
      attribs[i++] = EGL_DMA_BUF_PLANE1_FD_EXT;
      attribs[i++] = b->dbuf_fd;

      attribs[i++] = EGL_DMA_BUF_PLANE1_OFFSET_EXT;
      attribs[i++] = offsets[1];

      attribs[i++] = EGL_DMA_BUF_PLANE1_PITCH_EXT;
      attribs[i++] = pitches[1];
   }

   if (pitches[2]) {
      attribs[i++] = EGL_DMA_BUF_PLANE2_FD_EXT;
      attribs[i++] = b->dbuf_fd;

      attribs[i++] = EGL_DMA_BUF_PLANE2_OFFSET_EXT;
      attribs[i++] = offsets[2];

      attribs[i++] = EGL_DMA_BUF_PLANE2_PITCH_EXT;
      attribs[i++] = pitches[2];
   }

   attribs[i++] = EGL_NONE;

   EGLImage image = eglCreateImageKHR(dpy,
                                      EGL_NO_CONTEXT,
                                      EGL_LINUX_DMA_BUF_EXT,
                                      NULL, attribs);
   if (!image) {
      fprintf(stderr, "Failed to import fd %d\n", b->dbuf_fd);
      exit(1);
   }

   glGenTextures(1, &b->texture);
   glBindTexture(GL_TEXTURE_EXTERNAL_OES, b->texture);
   glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
   glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
   glEGLImageTargetTexture2DOES(GL_TEXTURE_EXTERNAL_OES, image);

   eglDestroyImageKHR(dpy, image);

   return 0;

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

   return -1;
}

MMAL_POOL_T* pool_create_drm(MMAL_PORT_T *port, struct buffer *buffers, int drmfd,
                             EGLDisplay dpy, EGLContext ctx)
{
   MMAL_POOL_T *pool;
   unsigned int i;

   pool = mmal_port_pool_create(port,
                                port->buffer_num,
                                0);

   for (i = 0; i < port->buffer_num; i++)
   {
      buffer_create(&buffers[i], drmfd, port, dpy, ctx);
      
      pool->header[i]->data = (uint8_t*)vcsm_vc_hdl_from_hdl(buffers[i].vcsm_handle);
      pool->header[i]->alloc_size = port->buffer_size;
      pool->header[i]->length = 0;
      buffers[i].mmal_buffer = pool->header[i];
   }

   return pool;
}

void buffer_destroy(int drmfd, struct buffer *buf)
{
   struct drm_mode_destroy_dumb gem_destroy;

   vcsm_free(buf->vcsm_handle);
   glDeleteTextures(1, &buf->texture);
   close(buf->dbuf_fd);

   memset(&gem_destroy, 0, sizeof gem_destroy);
   gem_destroy.handle = buf->bo_handle;
   ioctl(drmfd, DRM_IOCTL_MODE_DESTROY_DUMB, &gem_destroy);
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

static int drm_mmal_create_buffers(MMAL_PORT_T *port, struct buffer *buffers, int drmfd, MMAL_POOL_T **pool_out,
                                   EGLDisplay dpy, EGLContext ctx)
{
   MMAL_STATUS_T status;
   port->buffer_size = port->buffer_size_min;
   printf("Set buffer size to %d\n", port->buffer_size);
   status = mmal_port_format_commit(port);

   *pool_out = pool_create_drm(port, buffers, drmfd, dpy, ctx);

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


/**
 * Remove window border/decorations.
 */
static void
no_border( Display *dpy, Window w)
{
   static const unsigned MWM_HINTS_DECORATIONS = (1 << 1);
   static const int PROP_MOTIF_WM_HINTS_ELEMENTS = 5;

   typedef struct
   {
      unsigned long       flags;
      unsigned long       functions;
      unsigned long       decorations;
      long                inputMode;
      unsigned long       status;
   } PropMotifWmHints;

   PropMotifWmHints motif_hints;
   Atom prop, proptype;
   unsigned long flags = 0;

   /* setup the property */
   motif_hints.flags = MWM_HINTS_DECORATIONS;
   motif_hints.decorations = flags;

   /* get the atom for the property */
   prop = XInternAtom( dpy, "_MOTIF_WM_HINTS", True );
   if (!prop) {
      /* something went wrong! */
      return;
   }

   /* not sure this is correct, seems to work, XA_WM_HINTS didn't work */
   proptype = prop;

   XChangeProperty( dpy, w,                         /* display, window */
                    prop, proptype,                 /* property, type */
                    32,                             /* format: 32-bit datums */
                    PropModeReplace,                /* mode */
                    (unsigned char *) &motif_hints, /* data */
                    PROP_MOTIF_WM_HINTS_ELEMENTS    /* nelements */
                  );
}


/*
 * Create an RGB, double-buffered window.
 * Return the window and context handles.
 */
static void
make_window( Display *dpy, EGLDisplay egl_dpy, const char *name,
             int x, int y, int width, int height,
             Window *winRet, EGLContext *ctxRet, EGLSurface *surfRet)
{
   int scrnum = DefaultScreen( dpy );
   XSetWindowAttributes attr;
   unsigned long mask;
   Window root = RootWindow( dpy, scrnum );
   Window win;
   EGLContext ctx;
   bool fullscreen = false; /* Hook this up to a command line arg */

   if (fullscreen) {
      int scrnum = DefaultScreen(dpy);

      x = 0; y = 0;
      width = DisplayWidth(dpy, scrnum);
      height = DisplayHeight(dpy, scrnum);
   }

   static const EGLint attribs[] = {
      EGL_RED_SIZE, 1,
      EGL_GREEN_SIZE, 1,
      EGL_BLUE_SIZE, 1,
      EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
      EGL_NONE
   };
   EGLConfig config;
   EGLint num_configs;
   if (!eglChooseConfig(egl_dpy, attribs, &config, 1, &num_configs)) {
      printf("Error: couldn't get an EGL visual config\n");
      exit(1);
   }

   EGLint vid;
   if (!eglGetConfigAttrib(egl_dpy, config, EGL_NATIVE_VISUAL_ID, &vid)) {
      printf("Error: eglGetConfigAttrib() failed\n");
      exit(1);
   }

   XVisualInfo visTemplate = {
      .visualid = vid,
   };
   int num_visuals;
   XVisualInfo *visinfo = XGetVisualInfo(dpy, VisualIDMask,
                                         &visTemplate, &num_visuals);

   /* window attributes */
   attr.background_pixel = 0;
   attr.border_pixel = 0;
   attr.colormap = XCreateColormap( dpy, root, visinfo->visual, AllocNone);
   attr.event_mask = StructureNotifyMask | ExposureMask | KeyPressMask;
   /* XXX this is a bad way to get a borderless window! */
   mask = CWBackPixel | CWBorderPixel | CWColormap | CWEventMask;

   win = XCreateWindow( dpy, root, x, y, width, height,
                        0, visinfo->depth, InputOutput,
                        visinfo->visual, mask, &attr );

   if (fullscreen)
      no_border(dpy, win);

   /* set hints and properties */
   {
      XSizeHints sizehints;
      sizehints.x = x;
      sizehints.y = y;
      sizehints.width  = width;
      sizehints.height = height;
      sizehints.flags = USSize | USPosition;
      XSetNormalHints(dpy, win, &sizehints);
      XSetStandardProperties(dpy, win, name, name,
                              None, (char **)NULL, 0, &sizehints);
   }

   eglBindAPI(EGL_OPENGL_ES_API);

   static const EGLint ctx_attribs[] = {
      EGL_CONTEXT_CLIENT_VERSION, 2,
      EGL_NONE
   };
   ctx = eglCreateContext(egl_dpy, config, EGL_NO_CONTEXT, ctx_attribs );
   if (!ctx) {
      printf("Error: eglCreateContext failed\n");
      exit(1);
   }

   XFree(visinfo);

   XMapWindow(dpy, win);

   EGLSurface surf = eglCreateWindowSurface(egl_dpy, config,
                                            (void *)(uintptr_t)win, NULL);
   if (!surf) {
      printf("Error: eglCreateWindowSurface failed\n");
      exit(1);
   }

   if (!eglMakeCurrent(egl_dpy, surf, surf, ctx)) {
      printf("Error: eglCreateContext failed\n");
      exit(1);
   }

   *winRet = win;
   *ctxRet = ctx;
   *surfRet = surf;
}

static GLint
compile_shader(GLenum target, const char *source)
{
   GLuint s = glCreateShader(target);
	glShaderSource(s, 1, (const GLchar **) &source, NULL);
	glCompileShader(s);

   GLint ok;
   glGetShaderiv(s, GL_COMPILE_STATUS, &ok);

   if (!ok) {
		GLchar *info;
		GLint size;

		glGetShaderiv(s, GL_INFO_LOG_LENGTH, &size);
		info = malloc(size);

		glGetShaderInfoLog(s, size, NULL, info);
      fprintf(stderr, "Failed to compile shader: %s\n", info);

      fprintf(stderr, "source:\n%s", source);

      exit(1);
   }

   return s;
}

static GLint link_program(GLint vs, GLint fs)
{
	GLint prog = glCreateProgram();
   glAttachShader(prog, vs);
   glAttachShader(prog, fs);
   glLinkProgram(prog);

   GLint ok;
   glGetProgramiv(prog, GL_LINK_STATUS, &ok);
   if (!ok) {
      /* Some drivers return a size of 1 for an empty log.  This is the size
       * of a log that contains only a terminating NUL character.
       */
      GLint size;
      GLchar *info = NULL;
      glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &size);
      if (size > 1) {
         info = malloc(size);
         glGetProgramInfoLog(prog, size, NULL, info);
      }

      fprintf(stderr, "Failed to link: %s\n",
              (info != NULL) ? info : "<empty log>");
      exit(1);
   }

   return prog;
}

static void
gl_setup(void)
{
   const char *vs =
      "attribute vec4 pos;\n"
      "varying vec2 texcoord;\n"
      "\n"
      "void main() {\n"
      "  gl_Position = pos;\n"
      "  texcoord.x = (pos.x + 1.0) / 2.0;\n"
      "  texcoord.y = (pos.y + 1.0) / -2.0;\n"
      "}\n";
   GLint vs_s = compile_shader(GL_VERTEX_SHADER, vs);
   const char *fs =
      "#extension GL_OES_EGL_image_external : enable\n"
      "precision mediump float;\n"
      "uniform samplerExternalOES s;\n"
      "varying vec2 texcoord;\n"
      "void main() {\n"
      "  gl_FragColor = texture2D(s, texcoord);\n"
      "}\n";
   GLint fs_s = compile_shader(GL_FRAGMENT_SHADER, fs);
   GLint prog = link_program(vs_s, fs_s);

   glUseProgram(prog);

   static const float verts[] = {
      -1, -1,
      1, -1,
      1, 1,
      -1, 1,
   };
   glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, verts);
   glEnableVertexAttribArray(0);
}

static void
present_dmabuf(EGLDisplay dpy, EGLContext ctx, EGLSurface surf,
               struct buffer *buffer)
{
   glClearColor(0.5, 0.5, 0.5, 0.5);
   glClear(GL_COLOR_BUFFER_BIT);

   glBindTexture(GL_TEXTURE_EXTERNAL_OES, buffer->texture);
   glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
   eglSwapBuffers(dpy, surf);
}

static int
get_drm_fd(Display *dpy)
{
   xcb_connection_t *c = XGetXCBConnection(dpy);
   xcb_window_t root = RootWindow(dpy, DefaultScreen(dpy));
   int fd;

   const xcb_query_extension_reply_t *extension =
      xcb_get_extension_data(c, &xcb_dri3_id);
   if (!(extension && extension->present))
      return -1;

   xcb_dri3_open_cookie_t cookie =
      xcb_dri3_open(c, root, None);

   xcb_dri3_open_reply_t *reply = xcb_dri3_open_reply(c, cookie, NULL);
   if (!reply)
      return -1;

   if (reply->nfd != 1) {
      free(reply);
      return -1;
   }

   fd = xcb_dri3_open_reply_fds(c, reply)[0];
   free(reply);
   fcntl(fd, F_SETFD, fcntl(fd, F_GETFD) | FD_CLOEXEC);

   return fd;
}

int main(int argc, char **argv)
{
   MMAL_STATUS_T status = MMAL_EINVAL;
   MMAL_COMPONENT_T *decoder = NULL, *isp = NULL;
   MMAL_POOL_T *pool_in = NULL, *pool_out = NULL;
   MMAL_BOOL_T eos_sent = MMAL_FALSE, eos_received = MMAL_FALSE;
   struct drm_setup setup = {0};
   struct buffer buffers[MAX_BUFFERS];
   MMAL_BUFFER_HEADER_T *current_buffer = NULL;
   MMAL_CONNECTION_T *connection = NULL;
   unsigned int in_count = 0, conn_out_count = 0, conn_in_count = 0, out_count = 0;

   if (argc < 2)
   {
      fprintf(stderr, "usage: mmal-demo <filename>\n");
      return -1;
   }

   bcm_host_init();

   setup.dpy = XOpenDisplay(NULL);
   CHECK_CONDITION(!setup.dpy, "Couldn't open X display\n");

   setup.egl_dpy = eglGetDisplay(setup.dpy);
   CHECK_CONDITION(!setup.egl_dpy, "eglGetDisplay() failed\n");

   EGLint egl_major, egl_minor;
   CHECK_CONDITION(!eglInitialize(setup.egl_dpy, &egl_major, &egl_minor),
                   "Error: eglInitialize() failed\n");

   /* Get an authenticated DRM fd from X, since we're doing DMABUF
    * import from vc4 buffers to vcsm.  We'd really rather be having
    * vcsm export the MMAL buffers to us.
    */
   int drmfd = get_drm_fd(setup.dpy);

   vcsm_init();

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
   status = mmal_port_parameter_set_boolean(decoder->input[0], MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
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

   make_window(setup.dpy, setup.egl_dpy, "mmal-demo",
               0, 0, 800, 600, &setup.win, &setup.ctx, &setup.surf);

   gl_setup();

   pool_out = pool_create_drm(isp->output[0], buffers, drmfd,
                              setup.egl_dpy, setup.ctx);
   setup.out_fourcc = mmal_encoding_to_drm_fourcc(ENCODING_FOR_DRM);

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

               status = drm_mmal_create_buffers(isp->output[0], buffers, drmfd, &pool_out,
                                                setup.egl_dpy, setup.ctx);
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

            present_dmabuf(setup.egl_dpy, setup.ctx, setup.surf,
                           &buffers[index]);

            //Release buffer that was on the screen
            if (current_buffer)
               mmal_buffer_header_release(current_buffer);
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
