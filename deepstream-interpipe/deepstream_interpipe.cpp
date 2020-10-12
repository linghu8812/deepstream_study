/*
 * Copyright (c) 2018-2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <gst/gst.h>
#include <gst/rtsp/gstrtsptransport.h>
#include <glib.h>
#include <stdio.h>
#include "gstnvdsmeta.h"

#define MAX_DISPLAY_LEN 64

#define PGIE_CLASS_ID_VEHICLE 0
#define PGIE_CLASS_ID_PERSON 2

/* The muxer output resolution must be set if the input streams will be of
 * different resolution. The muxer will scale all the input frames to this
 * resolution. */
#define MUXER_BATCH_SIZE 4
#define MUXER_OUTPUT_WIDTH 1920
#define MUXER_OUTPUT_HEIGHT 1080

/* Muxer batch formation timeout, for e.g. 40 millisec. Should ideally be set
 * based on the fastest source's framerate. */
#define MUXER_BATCH_TIMEOUT_USEC 40000

#define PIPE_NUM 4
#define PIPE_NUM_2 8

static GstPad *blockpad = NULL;

gint frame_number = 0;
gchar pgie_classes_str[4][32] = {"Vehicle", "TwoWheeler", "Person",
                                 "Roadsign"};

/* osd_sink_pad_buffer_probe  will extract metadata received on OSD sink pad
 * and update params for drawing rectangle, object information etc. */

static GstPadProbeReturn
osd_sink_pad_buffer_probe(GstPad *pad, GstPadProbeInfo *info,
                          gpointer u_data)
{
  struct timeval *tv_infer = (struct timeval *)u_data;
  GstBuffer *buf = (GstBuffer *)info->data;
  guint num_rects = 0;
  NvDsObjectMeta *obj_meta = NULL;
  guint vehicle_count = 0;
  guint person_count = 0;
  guint frame_count = 0;
  NvDsMetaList *l_frame = NULL;
  NvDsMetaList *l_obj = NULL;
  NvDsDisplayMeta *display_meta = NULL;

  /* Update Inference tic tok */
  gettimeofday(tv_infer, NULL);
  g_print("Infer timestamp update %lu.%06lu\n", tv_infer->tv_sec, tv_infer->tv_usec);

  NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta(buf);

  for (l_frame = batch_meta->frame_meta_list; l_frame != NULL;
       l_frame = l_frame->next)
  {
    NvDsFrameMeta *frame_meta = (NvDsFrameMeta *)(l_frame->data);
    frame_count++;
    g_print("Frame number %d Batch id %u Source id %u\n", frame_meta->frame_num, frame_meta->batch_id, frame_meta->source_id);
    int offset = 0;
    for (l_obj = frame_meta->obj_meta_list; l_obj != NULL;
         l_obj = l_obj->next)
    {
      obj_meta = (NvDsObjectMeta *)(l_obj->data);
      if (obj_meta->class_id == PGIE_CLASS_ID_VEHICLE)
      {
        vehicle_count++;
        num_rects++;
      }
      if (obj_meta->class_id == PGIE_CLASS_ID_PERSON)
      {
        person_count++;
        num_rects++;
      }
    }
    display_meta = nvds_acquire_display_meta_from_pool(batch_meta);
    NvOSD_TextParams *txt_params = &display_meta->text_params[0];
    display_meta->num_labels = 1;
    txt_params->display_text = (char *)g_malloc0(MAX_DISPLAY_LEN);
    offset = snprintf(txt_params->display_text, MAX_DISPLAY_LEN, "Person = %d ", person_count);
    offset = snprintf(txt_params->display_text + offset, MAX_DISPLAY_LEN, "Vehicle = %d ", vehicle_count);

    /* Now set the offsets where the string should appear */
    txt_params->x_offset = 10;
    txt_params->y_offset = 12;

    /* Font , font-color and font-size */
    txt_params->font_params.font_name = "Serif";
    txt_params->font_params.font_size = 10;
    txt_params->font_params.font_color.red = 1.0;
    txt_params->font_params.font_color.green = 1.0;
    txt_params->font_params.font_color.blue = 1.0;
    txt_params->font_params.font_color.alpha = 1.0;

    /* Text background color */
    txt_params->set_bg_clr = 1;
    txt_params->text_bg_clr.red = 0.0;
    txt_params->text_bg_clr.green = 0.0;
    txt_params->text_bg_clr.blue = 0.0;
    txt_params->text_bg_clr.alpha = 1.0;

    nvds_add_display_meta_to_frame(frame_meta, display_meta);
  }

  g_print("Frame Number = %d Frame Count = %d Number of objects = %d "
          "Vehicle Count = %d Person Count = %d\n",
          frame_number, frame_count, num_rects, vehicle_count, person_count);
  frame_number++;
  return GST_PAD_PROBE_OK;
}

static gboolean
bus_call(GstBus *bus, GstMessage *msg, gpointer data)
{
  GMainLoop *loop = (GMainLoop *)data;
  g_print("GST_MESSAGE_TYPE(msg): %#x\n", GST_MESSAGE_TYPE(msg));
  switch (GST_MESSAGE_TYPE(msg))
  {
  case GST_MESSAGE_EOS:
    g_print("End of stream\n");
    g_main_loop_quit(loop);
    break;
  case GST_MESSAGE_ERROR:
  {
    gchar *debug;
    GError *error;
    gst_message_parse_error(msg, &error, &debug);
    g_printerr("ERROR from element %s: %s\n",
               GST_OBJECT_NAME(msg->src), error->message);
    if (debug)
      g_printerr("Error details: %s\n", debug);
    g_free(debug);
    g_error_free(error);
    g_main_loop_quit(loop);
    break;
  }
  default:
    break;
  }
  return TRUE;
}

static GstElement *
create_rtsp_source_pipeline(guint index, gchar *location)
{
  GstElement *pipeline = NULL /*, *uri_decode_bin = NULL*/;
  gchar pipeline_name[16] = {};
  gchar interpipe_name[16] = {};

  g_snprintf(pipeline_name, 15, "pipeline-%02d", index);
  g_snprintf(interpipe_name, 15, "interpipe-%02d", index);
  g_print("interpipe_name: %s\n", interpipe_name);
  /* Create a source GstBin to abstract this pipeline's content from the rest of the
   * pipeline */
  pipeline = gst_pipeline_new(pipeline_name);

  if (!pipeline)
  {
    g_printerr("One element could not be created. Exiting.\n");
    return pipeline;
  }

  GstElement *source = NULL, *queue = NULL, *parser = NULL, *decoder = NULL,
             *queue_d = NULL, *scale = NULL, *capsfilter = NULL, *interpipesink = NULL;

  source = gst_element_factory_make("rtspsrc", "source");
  queue = gst_element_factory_make("queue", "source-queue");
  parser = gst_element_factory_make("parsebin", "parser");
  decoder = gst_element_factory_make("avdec_h265", "h265-decoder");
  queue_d = gst_element_factory_make("queue", "decoder-queue");
  scale = gst_element_factory_make("videoscale", "scale");
  capsfilter = gst_element_factory_make("capsfilter", "capsfilter");
  interpipesink = gst_element_factory_make("interpipesink", interpipe_name);

  if (!source || !queue || !parser || !decoder || !queue_d || !scale || !capsfilter || !interpipesink)
  {
    g_printerr("One element could not be created. Exiting.\n");
    return NULL;
  }

  GstCaps *caps = gst_caps_new_simple("video/x-raw", "width", G_TYPE_INT, 1920, "height", G_TYPE_INT, 1080, NULL);
  g_object_set(G_OBJECT(capsfilter), "caps", caps, NULL);
  g_object_set(G_OBJECT(source), "location", location, "protocols", GST_RTSP_LOWER_TRANS_TCP, NULL);
  g_object_set(G_OBJECT(interpipesink), "sync", FALSE, "async", FALSE, NULL);

  gst_bin_add_many(GST_BIN(pipeline), source, queue, parser, decoder, queue_d, scale, capsfilter, interpipesink, NULL);
  gst_element_link_many(source, queue, parser, decoder, queue_d, scale, capsfilter, interpipesink, NULL);

  return pipeline;
}

static GstElement *
create_sink_pipeline(guint index, const gchar *listen_to_prefix, struct timeval *tv_infer)
{
  GstElement *pipeline = NULL /*, *uri_decode_bin = NULL*/;
  gchar pipeline_name[32] = {};
  gchar interpipe_name[32] = {};
  gchar interpipe_queue_name[32] = {};
  gchar listen_to[32] = {};

  g_snprintf(pipeline_name, 31, "sink-pipeline-%02d", index);
  g_snprintf(interpipe_name, 31, "interpipesrc-%02d", index);
  g_print("interpipe_name: %s\n", interpipe_name);
  /* Create a source GstBin to abstract this pipeline's content from the rest of the
   * pipeline */
  pipeline = gst_pipeline_new(pipeline_name);

  if (!pipeline)
  {
    g_printerr("[%d] One element could not be created. Exiting.\n", __LINE__);
    return pipeline;
  }

  GstElement *interpipesrc[16] = {NULL}, *queue[16] = {NULL};
  GstElement *nvvideoconvert = NULL, *autovideosink = NULL;
  GstElement *streammux = NULL, *pgie = NULL, *nvvidconv = NULL, *nvosd = NULL, *nvvidconv2sink = NULL;

  /* Create all interpipesrc & queue */
  for (size_t i = 0; i < PIPE_NUM; i++)
  {
    g_snprintf(interpipe_name, 31, "interpipesrc-%02lu", i);
    g_snprintf(interpipe_queue_name, 31, "interpipesrc-%02lu-queue", i);
    interpipesrc[i] = gst_element_factory_make("interpipesrc", interpipe_name);
    queue[i] = gst_element_factory_make("queue", interpipe_queue_name);
    if (!interpipesrc[i] || !queue[i])
    {
      g_printerr("[%d] One element could not be created. Exiting.\n", __LINE__);
      return NULL;
    }
  }

  nvvideoconvert = gst_element_factory_make("nvvideoconvert", "nvvideoconvert");
  autovideosink = gst_element_factory_make("fakesink", "autovideosink");

  if (!nvvideoconvert || !autovideosink)
  {
    g_printerr("[%d] One element could not be created. Exiting.\n", __LINE__);
    return NULL;
  }

  streammux = gst_element_factory_make("nvstreammux", "streammux");
  pgie = gst_element_factory_make("nvinfer", "pgie");
  nvvidconv = gst_element_factory_make("nvvideoconvert", "nvvidconv");
  nvvidconv2sink = gst_element_factory_make("nvvideoconvert", "nvvidconv2sink");
  nvosd = gst_element_factory_make("nvdsosd", "nvosd");

  if (!streammux || !pgie || !nvvidconv || !nvosd || !nvvidconv2sink)
  {
    g_printerr("[%d] One element could not be created. Exiting.\n", __LINE__);
    return NULL;
  }

  /* Set property listen-to of all interpipesrc */
  for (size_t i = 0; i < PIPE_NUM; i++)
  {
    g_snprintf(listen_to, 31, "interpipe-%02lu", i);
    g_print("%s listen-to %s\n", GST_ELEMENT_NAME(interpipesrc[i]), listen_to);
    g_object_set(G_OBJECT(interpipesrc[i]), "listen-to", listen_to, "is-live", TRUE, "allow-renegotiation", TRUE, "stream-sync", 0, NULL);
  }

  /* link all interpipesrc & queue one by one */
  for (size_t i = 0; i < PIPE_NUM; i++)
  {
    gst_bin_add_many(GST_BIN(pipeline), interpipesrc[i], queue[i], NULL);
    gst_element_link_many(interpipesrc[i], queue[i], NULL);
  }

  /* Set all the necessary properties of the streammux element,
   * the necessary ones are : */
  g_object_set(G_OBJECT(streammux), "batch-size", MUXER_BATCH_SIZE, NULL);
  g_object_set(G_OBJECT(streammux), "width", MUXER_OUTPUT_WIDTH, "height",
               MUXER_OUTPUT_HEIGHT,
               "batched-push-timeout", MUXER_BATCH_TIMEOUT_USEC, NULL);
  /* Set all the necessary properties of the nvinfer element,
   * the necessary ones are : */
  g_object_set(G_OBJECT(pgie),
               "config-file-path", "../dstest1_pgie_config.txt", NULL);

  gst_bin_add_many(GST_BIN(pipeline), nvvideoconvert, autovideosink, NULL);
  gst_bin_add_many(GST_BIN(pipeline), streammux, pgie, nvvidconv, nvosd, nvvidconv2sink, NULL);
  /* Link streammux and the element */
  gst_element_link_many(streammux, pgie, nvvidconv, nvosd, nvvidconv2sink, autovideosink, NULL);

  /* Link the interpipesrc queue to sink pads of streammux */
  for (size_t i = 0; i < PIPE_NUM; i++)
  {
    GstPad *sinkpad, *srcpad;
    gchar pad_name_sink[16] = "sink_0";
    gchar pad_name_src[16] = "src";

    g_snprintf(pad_name_sink, 15, "sink_%lu", i);
    g_print("%s of streammux\n", pad_name_sink);

    sinkpad = gst_element_get_request_pad(streammux, pad_name_sink);
    if (!sinkpad)
    {
      g_printerr("Streammux request sink pad failed. Exiting.\n");
      return NULL;
    }

    srcpad = gst_element_get_static_pad(queue[i], pad_name_src);
    if (!srcpad)
    {
      g_printerr("Decoder request src pad failed. Exiting.\n");
      return NULL;
    }

    if (gst_pad_link(srcpad, sinkpad) != GST_PAD_LINK_OK)
    {
      g_printerr("Failed to link decoder to stream muxer. Exiting.\n");
      return NULL;
    }

    gst_object_unref(sinkpad);
    gst_object_unref(srcpad);
  }

  /* Lets add probe to get informed of the meta data generated, we add probe to
   * the sink pad of the osd element, since by that time, the buffer would have
   * had got all the metadata. */
  GstPad *osd_sink_pad = NULL;
  osd_sink_pad = gst_element_get_static_pad(nvosd, "sink");
  if (!osd_sink_pad)
    g_print("Unable to get sink pad\n");
  else
    gst_pad_add_probe(osd_sink_pad, GST_PAD_PROBE_TYPE_BUFFER,
                      osd_sink_pad_buffer_probe, tv_infer, NULL);

  return pipeline;
}

int main(int argc, char *argv[])
{
  gchar *listen_to_1;
  gchar *listen_to_2;
  const gchar listen_to_prefix[] = "interpipe";
  const gchar *rtsp_cmd_template[] = {
    "rtspsrc location=rtsp://192.168.204.64:20150/live/573d2a9445704847adcc12c78bf7a83d_2 protocols=GST_RTSP_LOWER_TRANS_TCP ! queue ! parsebin ! nvv4l2decoder ! queue ! interpipesink name=%s sync=false async=false",
    "rtspsrc location=rtsp://192.168.204.64:20150/live/67f5b6dc277e4fcdae9bf078ae12a619_2 protocols=GST_RTSP_LOWER_TRANS_TCP ! queue ! parsebin ! nvv4l2decoder ! queue ! interpipesink name=%s sync=false async=false",
    "rtspsrc location=rtsp://192.168.204.64:20150/live/8a22dc8c92b54dd3bec0b2671e3cc578_2 protocols=GST_RTSP_LOWER_TRANS_TCP ! queue ! parsebin ! nvv4l2decoder ! queue ! interpipesink name=%s sync=false async=false"
  };
  gchar rtsp_cmd[256];
  size_t listen_to_num_base = 0;
  /* Inference tic tok */
  struct timeval tv_infer;
  struct timeval tv_now;
  gettimeofday(&tv_infer, NULL);
  /* Standard GStreamer initialization */
  gst_init(&argc, &argv);
  GstBus *bus = NULL;
  guint bus_watch_id;
  GstElement *interpipesrc[PIPE_NUM] = {NULL};
  GstElement *queue[PIPE_NUM] = {NULL};
  GstElement *pipe[PIPE_NUM_2] = {NULL};
  GMainLoop *loop = g_main_loop_new(NULL, FALSE);

  /* Create inference pipeline */
  GstElement *pipe_infer = create_sink_pipeline(0, listen_to_prefix, &tv_infer);

  bus = gst_pipeline_get_bus(GST_PIPELINE(pipe_infer));
  bus_watch_id = gst_bus_add_watch(bus, bus_call, loop);
  gst_object_unref(bus);

  /* Get all interpipesrc of inference pipeline */
  for (size_t i = 0; i < PIPE_NUM; i++)
  {
    gchar interpipe_name[32] = {};
    gchar interpipe_queue_name[32] = {};
    g_snprintf(interpipe_name, 31, "interpipesrc-%02lu", i);
    g_snprintf(interpipe_queue_name, 31, "interpipesrc-%02lu-queue", i);
    interpipesrc[i] = gst_bin_get_by_name((GstBin *)pipe_infer, interpipe_name);
    queue[i] = gst_bin_get_by_name((GstBin *)pipe_infer, interpipe_queue_name);
  }

  g_print("Ready...\n");

  sleep(1);
  g_print("Start Running...\n");
  for (size_t i = 0; i < 3; i++)
  {
    gchar interpipe_name[16] = {};
    g_snprintf(interpipe_name, 15, "%s-%02lu", listen_to_prefix, (listen_to_num_base + i) % (2 * MUXER_BATCH_SIZE));
    g_snprintf(rtsp_cmd, 255, rtsp_cmd_template[i], interpipe_name);
    g_print("%s\n", rtsp_cmd);
    pipe[i] = gst_parse_launch(rtsp_cmd, NULL);
    gst_element_set_state(pipe[i], GST_STATE_PLAYING);
  }
  
  gst_element_set_state(pipe_infer, GST_STATE_PLAYING);

  for (size_t j = 0; j < 100; j++)
  {
    sleep(5);
    g_print("Stop listen interpipe-XX...\n");
    /* Cut off all connection between rtsp streamer and inference pipeline */
    for (size_t i = 0; i < PIPE_NUM; i++, listen_to_num_base++)
    {
      g_object_set(interpipesrc[i], "listen-to", NULL, NULL);
    }

    /* Check that the buffer of the inference pipeline is empty or not */
    do
    {
      gettimeofday(&tv_now, NULL);
      g_print("tv_infer %lu.%06lu sec\n", tv_infer.tv_sec, tv_infer.tv_usec);
      g_print("tv_now %lu.%06lu sec\n", tv_now.tv_sec, tv_now.tv_usec);
      g_print("tv_tic toc %lu usec\n", (tv_now.tv_sec - tv_infer.tv_sec) * 1000000 + (tv_now.tv_usec - tv_infer.tv_usec));
      usleep(MUXER_BATCH_TIMEOUT_USEC);
    } while ((tv_now.tv_sec - tv_infer.tv_sec) * 1000000 + (tv_now.tv_usec - tv_infer.tv_usec) < MUXER_BATCH_TIMEOUT_USEC * 5);

    g_print("Pause and Restart Running...\n");
    /* launch rtsp streamer */
    for (size_t i = 0; i < 3; i++)
    {
      gchar interpipe_name[16] = {};
      g_snprintf(interpipe_name, 15, "%s-%02lu", listen_to_prefix, (listen_to_num_base + i) % (2 * MUXER_BATCH_SIZE));
      g_snprintf(rtsp_cmd, 255, rtsp_cmd_template[i], interpipe_name);
      g_print("%s\n", rtsp_cmd);
      pipe[(listen_to_num_base + i) % (2 * MUXER_BATCH_SIZE)] = gst_parse_launch(rtsp_cmd, NULL);
      gst_element_set_state(pipe[(listen_to_num_base + i) % (2 * MUXER_BATCH_SIZE)], GST_STATE_PLAYING);
    }
    /* destroy older rtsp streamer */
    for (size_t i = 0; i < PIPE_NUM; i++)
    {
      if (pipe[(listen_to_num_base + i + PIPE_NUM) % (2 * MUXER_BATCH_SIZE)]) {
        gst_element_set_state(pipe[(listen_to_num_base + i + PIPE_NUM) % (2 * MUXER_BATCH_SIZE)], GST_STATE_NULL);
      }
    }
    for (size_t i = 0; i < PIPE_NUM; i++)
    {
      if (pipe[(listen_to_num_base + i + PIPE_NUM) % (2 * MUXER_BATCH_SIZE)]) {
        gst_object_unref(GST_OBJECT(pipe[(listen_to_num_base + i + PIPE_NUM) % (2 * MUXER_BATCH_SIZE)]));
        pipe[(listen_to_num_base + i + PIPE_NUM) % (2 * MUXER_BATCH_SIZE)] = NULL;
      }
    }

    sleep(3);
    g_print("Start listen interpipe-XX...\n");
    for (size_t i = 0; i < PIPE_NUM; i++)
    {
      gchar interpipe_name[16] = {};
      g_snprintf(interpipe_name, 15, "%s-%02lu", listen_to_prefix, (listen_to_num_base + i) % (2 * MUXER_BATCH_SIZE));
      g_print("%s listen-to %s\n", GST_ELEMENT_NAME(interpipesrc[i]), interpipe_name);
      g_object_set(interpipesrc[i], "listen-to", interpipe_name, NULL);
    }
  }

  g_print("Running...\n");
  g_main_loop_run(loop);

  /* Out of the main loop, clean up nicely */
  g_print("Returned, stopping playback\n");
  gst_element_set_state(pipe_infer, GST_STATE_NULL);
  for (size_t i = 0; i < PIPE_NUM_2; i++)
  {
    if (pipe[i]) {
      gst_element_set_state(pipe[i], GST_STATE_NULL);
    }
  }
  g_print("Deleting pipeline\n");
  gst_object_unref(GST_OBJECT(pipe_infer));
  for (size_t i = 0; i < PIPE_NUM_2; i++)
  {
    if (pipe[i]) {
      gst_object_unref(GST_OBJECT(pipe[i]));
      pipe[i] = NULL;
    }
  }
  // g_source_remove(bus_watch_id);
  g_free(listen_to_1);
  g_free(listen_to_2);
  g_main_loop_unref(loop);
}
