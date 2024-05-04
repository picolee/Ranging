/**
 *   @file  mss_main.c
 *
 *   @brief
 *      This is the main file which implements the millimeter wave Demo
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2019 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** @mainpage Millimeter Wave (mmw) Demo for XWR68XX
 * [TOC]
 *  @section intro_sec Introduction
 *
 *  @image html toplevel.png
 *
 *  The millimeter wave demo shows some of the capabilities of the XWR68xx SoC
 *  using the drivers in the mmWave SDK (Software Development Kit).
 *  It allows user to specify the chirping profile and displays the detected
 *  objects and other information in real-time.
 *
 *  Following is a high level description of the features of this demo:
 *  - Be able to specify desired chirping profile through command line interface (CLI)
 *    on a UART port or through the TI Gallery App - **mmWave Demo Visualizer** -
 *    that allows user to provide a variety of profile configurations via the UART input port
 *    and displays the streamed detected output from another UART port in real-time,
 *    as seen in picture above.
 *  - Some sample profile configurations have been provided in the demo directory that can be
 *    used with CLI directly or via **mmWave Demo Visualizer** under following directory:
 *    @verbatim mmw/profiles @endverbatim
 *  - Do 1D, 2D, CFAR, Azimuth and Elevation processing and stream out velocity
 *    and three spatial coordinates (x,y,z) of the detected objects in real-time.
 *    The demo can also be configured to do 2D only detection (velocity and x,y coordinates).
 *  - Various display options besides object detection like azimuth heat map and
 *    Doppler-range heat map.
 *
 *  @section limit Limitations
 *  - Because of UART speed limit (< 1 Mbps), the frame time is more restrictive.
 *    For example, for the azimuth and Doppler heat maps for 256 FFT range and
 *    16 point FFT Doppler, it takes about 200 ms to transmit.
 *  - For most boards, a range bias of few centimeters has been observed. User can estimate
 *    the range bias on their board and correct using the calibration procedure
 *    described in @ref Calibration_section.
 *
 *  @section systemFlow System Execution Flow
 *  The millimeter wave demo runs on R4F (MSS) and DSP(DSS). Following diagram shows the system
 *  execution flow
 *
 *  @image html system_flow.png "System Execution Flow"
 *
 *  @section tasks Software Tasks
 *    The demo consists of the following (SYSBIOS) tasks:
 *
 *    **MSS**
 *    - @ref Ranging_initTask. This task is created/launched by @ref main and is a
 *      one-time active task whose main functionality is to initialize drivers (\<driver\>_init),
 *      MMWave module (MMWave_init), DPM module (DPM_init), open UART and data
 *      path related drivers (ADCBUF), and create/launch the following tasks
 *      (the @ref CLI_task is launched indirectly by calling @ref CLI_open).
 *    - @ref CLI_task. This command line interface task provides a simplified 'shell' interface
 *      which allows the configuration of the BSS via the mmWave interface (MMWave_config).
 *      It parses input CLI configuration commands like chirp profile and GUI configuration.
 *      When sensor start CLI command is parsed, all actions related to starting sensor and
 *      starting the processing the data path are taken.
 *      When sensor stop CLI command is parsed, all actions related to stopping the sensor and
 *      stopping the processing of the data path are taken
 *    - @ref Ranging_mmWaveCtrlTask. This task is used to provide an execution
 *      context for the mmWave control, it calls in an endless loop the MMWave_execute API.
 *    - @ref mmwDemo_mssDPMTask. This task is used to provide an execution context for 
 *      DPM (Data Path Manager) execution, it calls in an endless loop. There is no DPC registered
 *      with DPM.
 *
 *    **DSS**
 *    - @ref Ranging_initTask. This task is created/launched by @ref main and is a
 *      one-time active task whose main functionality is to initialize drivers (\<driver\>_init),
 *      DPM module (DPM_init), data path related drivers (EDMA, HWA), and
 *      create/launch the following tasks.
 *    - @ref Ranging_DPC_Ranging_dpmTask. This task is used to provide an execution
 *      context for DPM (Data Path Manager) execution, it calls in an endless loop
 *      the DPM_execute API. In this context, all of the registered object detection
 *      DPC (Data Path Chain) APIs like configuration, control and execute will
 *      take place. In this task. When the DPC's execute API produces the detected objects and other
 *      results, they are reported to MSS where they are transmitted out of the UART port for display 
 *      using the visualizer.
 *
 *   @section datapath Data Path
 *   @image html datapath_overall.png "Top Level Data Path Processing Chain"
 *   \n
 *   \n
 *   @image html datapath_overall_timing.png "Top Level Data Path Timing"
 *
 *   The data path processing consists of taking ADC samples as input and producing
 *   detected objects (point-cloud and other information) to be shipped out of
 *   UART port to the PC. The algorithm processing is realized using
 *   the DPM registered Object Detection DPC. The details of the processing in DPC
 *   can be seen from the following doxygen documentation:
 *   **Chirp processing DPC(objdetrangehwa)**
 *   @verbatim
      ti/datapath/dpc/objectdetection/objdetrangehwa/docs/doxygen/html/index.html
     @endverbatim
 *   **inter frame processing DPC(objdetdsp)**
 *   @verbatim
      ti/datapath/dpc/objectdetection/objdetdsp/docs/doxygen/html/index.html
     @endverbatim
 *
 *  @section output Output information sent to host
 *      @subsection output_general Output Packet
 *      Output packets with the detection information are sent out every frame
 *      through the UART. Each packet consists of the header @ref Ranging_output_message_header_t
 *      and the number of TLV items containing various data information with
 *      types enumerated in @ref Ranging_output_message_type_e. The numerical values
 *      of the types can be found in @ref mmw_output.h. Each TLV
 *      item consists of type, length (@ref Ranging_output_message_tl_t) and payload information.
 *      The structure of the output packet is illustrated in the following figure.
 *      Since the length of the packet depends on the number of detected objects
 *      it can vary from frame to frame. The end of the packet is padded so that
 *      the total packet length is always multiple of 32 Bytes.
 *
 *      @image html output_packet_uart.png "Output packet structure sent to UART"
 *
 *      The following subsections describe the structure of each TLV.
 *
 *      @subsection tlv1 List of detected objects
 *       Type: (@ref MMWDEMO_OUTPUT_MSG_DETECTED_POINTS)
 *
 *       Length: (Number of detected objects) x (size of @ref DPIF_PointCloudCartesian_t)
 *
 *       Value: Array of detected objects. The information of each detected object
 *       is as per the structure @ref DPIF_PointCloudCartesian_t. When the number of
 *       detected objects is zero, this TLV item is not sent. The maximum number of
 *       objects that can be detected in a sub-frame/frame is @ref DPC_OBJDET_MAX_NUM_OBJECTS.
 *
 *       The orientation of x,y and z axes relative to the sensor is as per the
 *       following figure.
 *
 *        @image html coordinate_geometry.png "Coordinate Geometry"
 *
 *       The whole detected objects TLV structure is illustrated in figure below.
 *
 *       @image html detected_objects_tlv.png "Detected objects TLV"
 *
 *      @subsection tlv2 Range profile
 *       Type: (@ref MMWDEMO_OUTPUT_MSG_RANGE_PROFILE)
 *
 *       Length: (Range FFT size) x (size of uint16_t)
 *
 *       Value: Array of profile points at 0th Doppler (stationary objects).
 *       The points represent the sum of log2 magnitudes of received antennas
 *       expressed in Q9 format.
 *
 *      @subsection tlv3 Noise floor profile
 *       Type: (@ref MMWDEMO_OUTPUT_MSG_NOISE_PROFILE)
 *
 *       Length: (Range FFT size) x (size of uint16_t)
 *
 *       Value: This is the same format as range profile but the profile
 *       is at the maximum Doppler bin (maximum speed
 *       objects). In general for stationary scene, there would be no objects or clutter at
 *       maximum speed so the range profile at such speed represents the
 *       receiver noise floor.
 *
 *      @subsection tlv4 Azimuth static heatmap
 *       Type: (@ref MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP)
 *
 *       Length: (Range FFT size) x (Number of virtual antennas) (size of @ref cmplx16ImRe_t_)
 *
 *       Value: Array @ref DPU_AoAProcHWA_HW_Resources::azimuthStaticHeatMap. The antenna data
 *       are complex symbols, with imaginary first and real second in the following order:\n
 *       @verbatim
             Imag(ant 0, range 0), Real(ant 0, range 0),...,Imag(ant N-1, range 0),Real(ant N-1, range 0)
             ...
             Imag(ant 0, range R-1), Real(ant 0, range R-1),...,Imag(ant N-1, range R-1),Real(ant N-1, range R-1)
         @endverbatim
 *       Based on this data the static azimuth heat map is constructed by the GUI
 *       running on the host.
 *
 *      @subsection tlv5 Range/Doppler heatmap
 *       Type: (@ref MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP)
 *
 *       Length: (Range FFT size) x (Doppler FFT size) (size of uint16_t)
 *
 *       Value: Detection matrix @ref DPIF_DetMatrix::data.
 *       The order is : \n
 *       @verbatim
            X(range bin 0, Doppler bin 0),...,X(range bin 0, Doppler bin D-1),
            ...
            X(range bin R-1, Doppler bin 0),...,X(range bin R-1, Doppler bin D-1)
         @endverbatim
 *
 *      @subsection tlv6 Stats information
 *       Type: (@ref MMWDEMO_OUTPUT_MSG_STATS )
 *
 *       Length: (size of @ref Ranging_output_message_stats_t)
 *
 *       Value: Timing information as per @ref Ranging_output_message_stats_t.
 *       See timing diagram below related to the stats.
 *
 *      @image html processing_timing.png "Processing timing"
 *
 *       Note:
 *       -#  The @ref Ranging_output_message_stats_t::interChirpProcessingMargin is not
 *           computed (it is always set to 0). This is because there is no CPU involvement
 *           in the 1D processing (only HWA and EDMA are involved), and it is not possible to
 *           know how much margin is there in chirp processing without CPU being notified
 *           at every chirp when processing begins (chirp event) and when the HWA-EDMA
 *           computation ends. The CPU is intentionally kept free during 1D processing
 *           because a real application may use this time for doing some post-processing
 *           algorithm execution.
 *       -#  While the
 *           @ref Ranging_output_message_stats_t::interFrameProcessingTime reported
 *           will be of the current sub-frame/frame,
 *           the @ref Ranging_output_message_stats_t::interFrameProcessingMargin and
 *           @ref Ranging_output_message_stats_t::transmitOutputTime
 *           will be of the previous sub-frame (of the same
 *           @ref Ranging_output_message_header_t::subFrameNumber as that of the
 *           current sub-frame) or of the previous frame.
 *       -#  The @ref Ranging_output_message_stats_t::interFrameProcessingMargin excludes
 *           the UART transmission time (available as
 *           @ref Ranging_output_message_stats_t::transmitOutputTime). This is done
 *           intentionally to inform the user of a genuine inter-frame processing margin
 *           without being influenced by a slow transport like UART, this transport time
 *           can be significantly longer for example when streaming out debug information like
 *           heat maps. Also, in a real product deployment, higher speed interfaces (e.g LVDS)
 *           are likely to be used instead of UART. User can calculate the margin
 *           that includes transport overhead (say to determine the max frame rate
 *           that a particular demo configuration will allow) using the stats
 *           because they also contain the UART transmission time.
 *
 *      The CLI command \"guMonitor\" specifies which TLV element will
 *      be sent out within the output packet. The arguments of the CLI command are stored
 *      in the structure @ref Ranging_GuiMonSel_t.
 *
 *      @subsection tlv7 Side information of detected objects
 *       Type: (@ref MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO)
 *
 *       Length: (Number of detected objects) x (size of @ref DPIF_PointCloudSideInfo_t)
 *
 *       Value: Array of detected objects side information. The side information
 *       of each detected object is as per the structure @ref DPIF_PointCloudSideInfo_t).
 *       When the number of detected objects is zero, this TLV item is not sent.
 *
 *      @subsection tlv9 Temperature Stats
 *       Type: (@ref MMWDEMO_OUTPUT_MSG_TEMPERATURE_STATS)
 *
 *       Length: (size of @ref Ranging_temperatureStats_t)
 *
 *       Value: Structure of detailed temperature report as obtained from Radar front end.
 *       @ref Ranging_temperatureStats_t::tempReportValid is set to return value of
 *       rlRfGetTemperatureReport(). If @ref Ranging_temperatureStats_t::tempReportValid is 0,
 *       values in @ref Ranging_temperatureStats_t::temperatureReport are valid else they should
 *       be ignored. This TLV is sent along with Stats TLV described in @ref tlv6 
 *
 *  @section Calibration_section Range Bias and Rx Channel Gain/Phase Measurement and Compensation
 *
 *     Because of imperfections in antenna layouts on the board, RF delays in SOC, etc,
 *     there is need to calibrate the sensor to compensate for bias in the range estimation and
 *     receive channel gain and phase imperfections. The following figure illustrates
 *     the calibration procedure.
 *
 *     @anchor Figure_calibration_ladder_diagram
 *     @image html calibration_ladder_diagram.png "Calibration procedure ladder diagram"
 *
 *      The calibration procedure includes the following steps:
 *     -# Set a strong target like corner reflector at the distance of X meter
 *     (X less than 50 cm is not recommended) at boresight.
 *     -# Set the following command
 *     in the configuration profile in .../profiles/profile_calibration.cfg,
 *     to reflect the position X as follows:
 *     @verbatim
       measureRangeBiasAndRxChanPhase 1 X D
       @endverbatim
 *     where D (in meters) is the distance of window around X where the peak will be searched.
 *     The purpose of the search window is to allow the test environment from not being overly constrained
 *     say because it may not be possible to clear it of all reflectors that may be stronger than the one used
 *     for calibration. The window size is recommended to be at least the distance equivalent of a few range bins.
 *     One range bin for the calibration profile (profile_calibration.cfg) is about 5 cm.
 *     The first argument "1" is to enable the measurement. The stated configuration
 *     profile (.cfg) must be used otherwise the calibration may not work as expected
 *     (this profile ensures all transmit and receive antennas are engaged among
 *     other things needed for calibration).
 *     -# Start the sensor with the configuration file.
 *     -# In the configuration file, the measurement is enabled because of which
 *     the DPC will be configured to perform the measurement and generate
 *     the measurement result (@ref DPU_AoAProc_compRxChannelBiasCfg_t) in its
 *     result structure (@ref DPC_Ranging_ExecuteResult_t::compRxChanBiasMeasurement),
 *     the measurement results are written out on the CLI port (@ref Ranging_measurementResultOutput)
 *     in the format below:
 *     @verbatim
       compRangeBiasAndRxChanPhase <rangeBias> <Re(0,0)> <Im(0,0)> <Re(0,1)> <Im(0,1)> ... <Re(0,R-1)> <Im(0,R-1)> <Re(1,0)> <Im(1,0)> ... <Re(T-1,R-1)> <Im(T-1,R-1)>
       @endverbatim
       For details of how DPC performs the measurement, see the DPC documentation.
 *     -# The command printed out on the CLI now can be copied and pasted in any
 *     configuration file for correction purposes. This configuration will be
 *     passed to the DPC for the purpose  of applying compensation during angle
 *     computation, the details of this can be seen in the DPC documentation.
 *     If compensation is not desired,
 *     the following command should be given
 *     @verbatim
       compRangeBiasAndRxChanPhase 0.0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0
       @endverbatim
 *     Above sets the range bias to 0 and
 *     the phase coefficients to unity so that there is no correction. Note
 *     the two commands must always be given in any configuration file, typically
 *     the measure commmand will be disabled when the correction command is the
 *     desired one.
 *
 *  @section LVDSStreamingNotes Streaming data over LVDS
 *
 *    The LVDS streaming feature enables the streaming of HW data (a combination of ADC/CP/CQ data)
 *    and/or user specific SW data through LVDS interface.
 *    The streaming is done mostly by the CBUFF and EDMA peripherals with minimal CPU intervention.
 *    The streaming is configured through the @ref Ranging_LvdsStreamCfg_t CLI command which
 *    allows control of HSI header, enable/disable of HW and SW data and data format choice for the HW data.
 *    The choices for data formats for HW data are:
 *      - @ref MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED
 *      - @ref MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_ADC
 *      - @ref MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_CP_ADC_CQ
 *
 *    In order to see the high-level data format details corresponding to the above
 *    data format configurations, refer to the corresponding slides in ti/drivers/cbuff/docs/CBUFF_Transfers.pptx
 *
 *    When HW data LVDS streaming is enabled, the ADC/CP/CQ data is streamed per
 *    chirp on every chirp event. When SW data
 *    streaming is enabled, it is streamed during inter-frame period after the
 *    list of detected objects for that frame is computed.
 *    The SW data streamed every frame/sub-frame is composed of the following in time:
 *    -# HSI header (@ref HSIHeader_t): refer to HSI module for details.
 *    -# User data header: @ref Ranging_LVDSUserDataHeader
 *    -# User data payloads:
 *      -# Point-cloud information as a list : @ref DPIF_PointCloudCartesian_t x number of detected objects
 *      -# Point-cloud side information as a list : @ref DPIF_PointCloudSideInfo_t x number of detected objects
 *
 *    The format of the SW data streamed is shown in the following figure:
 *      @image html lvds_sw_data_format.png "LVDS SW Data format"
 *
 *    Note:
 *    -# Only single-chirp formats are allowed, multi-chirp is not supported.
 *    -# When number of objects detected in frame/sub-frame is 0, there is no
 *       transmission beyond the user data header.
 *    -# For HW data, the inter-chirp duration should be sufficient to stream out the desired amount
 *       of data. For example, if the HW data-format is ADC and HSI header is enabled,
 *       then the total amount of data generated per chirp
 *       is:\n
 *        (numAdcSamples * numRxChannels * 4 (size of complex sample) +
 *       52 [sizeof(HSIDataCardHeader_t) + sizeof(HSISDKHeader_t)] )
 *       rounded up to multiples of 256 [=sizeof(HSIHeader_t)] bytes.\n
 *       The chirp time Tc in us = idle time + ramp end time in the profile configuration.
 *       For n-lane LVDS with each lane at a maximum of B Mbps,\n
 *       maximum number of bytes that can be send per chirp = Tc * n * B / 8 which
 *       should be greater than the total amount of data generated per chirp i.e\n
 *       Tc * n * B / 8 >= round-up(numAdcSamples * numRxChannels * 4 + 52, 256). \n
 *       E.g if n = 2, B = 600 Mbps, idle time = 7 us, ramp end time = 44 us, numAdcSamples = 512,
 *       numRxChannels = 4, then 7650 >= 8448 is violated so this configuration will not work.
 *       If the idle-time is doubled in the above example, then we have 8700 > 8448, so
 *       this configuration will work.
 *    -# For SW data, the number of bytes to transmit each sub-frame/frame is:\n
 *       52 [sizeof(HSIDataCardHeader_t) + sizeof(HSISDKHeader_t)] +
 *       sizeof(Ranging_LVDSUserDataHeader_t) [=8] + \n
 *       number of detected objects (Nd) * { sizeof(DPIF_PointCloudCartesian_t) [=16] + sizeof(DPIF_PointCloudSideInfo_t) [=4] }
 *       rounded up to multiples of 256 [=sizeof(HSIHeader_t)] bytes.\n
 *       or X = round-up(60 + Nd * 20, 256). So the time to transmit this data will be \n
 *       X * 8 / (n*B) us. The maximum number of objects (Ndmax) that can be
 *       detected is defined in the DPC (@ref DPC_OBJDET_MAX_NUM_OBJECTS). So if
 *       Ndmax = 500, then time to transmit SW data is 68 us. Because we parallelize
 *       this transmission with the much slower UART transmission, and because UART transmission
 *       is also sending at least the same amount of information as the LVDS, the
 *       LVDS transmission time will not add any burdens on the processing budget
 *       beyond the overhead of reconfiguring and activating the CBUFF session
 *       (this overhead is likely bigger than the time to transmit).
 *    -# The total amount of data to be transmitted in a HW or SW packet must be greater than the
 *       minimum required by CBUFF, which is 64 bytes or 32 CBUFF Units (this is the definition
 *       CBUFF_MIN_TRANSFER_SIZE_CBUFF_UNITS in the CBUFF driver implementation).
 *       If this threshold condition is violated, the CBUFF driver will return an error during
 *       configuration and the demo will generate a fatal exception as a result.
 *       When HSI header is enabled, the total transfer size is ensured to be at least
 *       256 bytes, which satisfies the minimum. If HSI header is disabled, for the HW session,
 *       this means that numAdcSamples * numRxChannels * 4 >= 64. Although mmwavelink
 *       allows minimum number of ADC samples to be 2, the demo is supported
 *       for numAdcSamples >= 64. So HSI header is not required to be enabled for HW only case.
 *       But if SW session is enabled, without the HSI header, the bytes in each
 *       packet will be 8 + Nd * 20. So for frames/sub-frames where Nd < 3, the
 *       demo will generate exception. Therefore HSI header must be enabled if SW is enabled,
 *       this is checked in the CLI command validation.
 *
 *   @subsection lvdsImpl Implementation Notes
 *
 *    -# The LVDS implementation is mostly present in mmw_lvds_stream.h and
 *      mmw_lvds_stream.c with calls in mss_main.c. Additionally HSI
 *      clock initialization is done at first time sensor start using @ref Ranging_mssSetHsiClk.
 *    -# EDMA channel resources for CBUFF/LVDS are in the global resource file
 *      (mmw_res.h, see @ref resourceAlloc) along with other EDMA resource allocation.
 *      The user data header and two user payloads are configured as three user buffers in the CBUFF driver.
 *      Hence SW allocation for EDMA provides for three sets of EDMA resources as seen in
 *      the SW part (swSessionEDMAChannelTable[.]) of @ref Ranging_LVDSStream_EDMAInit.
 *      The maximum number of HW EDMA resources are needed for the
 *      data-format @ref MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_CP_ADC_CQ,
 *      which as seen in the corresponding slide in ti\drivers\cbuff\docs\CBUFF_Transfers.pptx is
 *      12 channels (+ shadows) including the 1st special CBUFF EDMA event channel
 *      which CBUFF IP generates to the EDMA, hence the HW part
 *      (hwwSessionEDMAChannelTable[.]) of @ref Ranging_LVDSStream_EDMAInit
 *      has 11 table entries.
 *    -# Although the CBUFF driver is configured for two sessions (hw and sw),
 *      at any time only one can be active. So depending on the LVDS CLI configuration
 *      and whether advanced frame or not, there is logic to activate/deactivate
 *      HW and SW sessions as necessary.
 *    -# The CBUFF session (HW/SW) configure-create and delete
 *      depends on whether or not re-configuration is required after the
 *      first time configuration.
 *      -# For HW session, re-configuration is done during sub-frame switching to
 *        re-configure for the next sub-frame but when there is no advanced frame
 *        (number of sub-frames = 1), the HW configuration does not need to
 *        change so HW session does not need to be re-created.
 *      -# For SW session, even though the user buffer start addresses and sizes of
 *        headers remains same, the number of detected objects which determines the
 *        sizes of some user buffers changes from one sub-frame/frame to another sub-frame/frame.
 *        Therefore SW session needs to be recreated every sub-frame/frame.
 *    -# User may modify the application software to transmit different information than point-cloud
 *      in the SW data e.g radar cube data (output of range DPU). However the CBUFF also has
 *      a maximum link list entry size limit of 0x3FFF CBUFF units or 32766 bytes.
 *      This means it is the limit for each user buffer entry
 *      [there are maximum of 3 entries -1st used for user data header, 2nd
 *      for point-cloud and 3rd for point-cloud side information]. During session creation,
 *      if this limit is exceeded, the CBUFF will return an error (and demo will in turn
 *      generate an exception). A single physical buffer of say size 50000 bytes
 *      may be split across two user buffers by providing one user buffer with
 *      (address, size) = (start address, 25000) and 2nd user buffer with
 *      (address, size) = (start address + 25000, 25000), beyond
 *      this two (or three if user data header is also replaced) limit,
 *      the user will need to create and activate (and wait for completion)
 *      the SW session multiple times to accomplish the transmission.
 *
 *    The following figure shows a timing diagram for the LVDS streaming
 *    (the figure is not to scale as actual durations will vary based on configuration).
 *
 *      @image html lvdstiming.png "LVDS timing diagram"
 *
 *  @section bypassCLI How to bypass CLI
 *
 *    Re-implement the file mmw_cli.c as follows:
 *
 *    -# @ref Ranging_CLIInit should just create a task with input taskPriority.
 *       Lets say the task is called "Ranging_sensorConfig_task".
 *    -# All other functions are not needed
 *    -# Implement the Ranging_sensorConfig_task as follows:
 *       - Fill gMmwMssMCB.cfg.openCfg
 *       - Fill gMmwMssMCB.cfg.ctrlCfg
 *       - Add profiles and chirps using @ref MMWave_addProfile and @ref MMWave_addChirp functions
 *       - Call @ref Ranging_CfgUpdate for every offset in @ref configStoreOffsets
 *         (MMWDEMO_xxx_OFFSET in mmw_mss.h)
 *       - Fill gMmwMssMCB.objDetCommonCfg.preStartCommonCfg
 *       - Call @ref Ranging_openSensor
 *       - Call @ref Ranging_configSensor
 *       - Call @ref Ranging_startSensor (One can use helper function
 *         @ref Ranging_isAllCfgInPendingState to know if all dynamic config was provided)
 *
 *  @section resourceAlloc Hardware Resource Allocation
 *    The object detection DPCs(objdetrangehwa and objdetdsp) need to configure the DPUs 
 *    hardware resources (HWA, EDMA etc). The resource partitioning is shown to be in
 *    the ownership of the demo. This is to illustrate the general
 *    case of resource allocation across DPCs and/or demo's own
 *    processing that is post-DPC processing. This partitioning
 *    can be seen in the mmw_res.h file. This file is passed as a compiler command line
 *    define @verbatim "--define=APP_RESOURCE_FILE="<ti/demo/xwr68xx/mmw/mmw_res.h>" @endverbatim 
 *    in mmw_mss.mak and mmw_dss.mak when building the DPC sources as part of building the demo application and
 *    is referred in object detection DPC sources where needed as
 *    @verbatim #include APP_RESOURCE_FILE @endverbatim
 *
 *  @section demoDesignNotes Design Notes
 *  
 *  Due to the limitation of DPM local queue size, for certain DPM functions such as @ref DPM_start, @ref DPM_stop and
 *  some of the DPC control through @ref DPM_ioctl, semaphores are used to sync between calling task and function
 *  @ref Ranging_DPC_Ranging_reportFxn. So that it won't cause DPM crash because of running out of
 *  DPM local queues. The following diagram demonstrates the example calling flow for blocking @ref DPM_ioctl() function call.
 *  Non-blocking @ref DPM_ioctl is also shown for comparison.
 *
 *  @image html dpm_ioctl_handling.png "DPM_ioctl calling flow"
 *
 *  There are DPM report functions on both MSS and DSS for the same DPM_Report. However the sequence is not guaranteed between the
 *  two cores.
 *
 *  @section memoryUsage Memory Usage
 *  @subsection memUsageSummary Memory usage summary
 *    The table below shows the usage of various memories available on the device across
 *    the demo application and other SDK components. The table is generated using the demo's
 *    map file and applying some mapping rules to it to generate a condensed summary.
 *    The numeric values shown here represent bytes.
 *
 *    **MSS:**
 *    For the mapping rules, please refer to <a href="../../demo_mss_mapping.txt">demo_mss_mapping.txt</a>.
 *    Refer to the <a href="../../xwr68xx_mmw_demo_mss_mem_analysis_detailed.txt">xwr68xx_mmw_demo_mss_mem_analysis_detailed.txt</a>
 *    for detailed analysis of the memory usage across drivers and control/alg components on MSS and to
 *    <a href="../../demo_mss_mapping_detailed.txt">demo_mss_mapping_detailed.txt</a>
 *    for detailed mapping rules .
 *
 *  \include xwr68xx_mmw_demo_mss_mem_analysis.txt
 *
 *    **DSS:**
 *    For the mapping rules, please refer to <a href="../../demo_dss_mapping.txt">demo_dss_mapping.txt</a>.
 *    Refer to the <a href="../../xwr68xx_mmw_demo_dss_mem_analysis_detailed.txt">xwr68xx_mmw_demo_dss_mem_analysis_detailed.txt</a>
 *    for detailed analysis of the memory usage across drivers and control/alg components on DSS and to
 *    <a href="../../demo_dss_mapping_detailed.txt">demo_dss_mapping_detailed.txt</a>
 *    for detailed mapping rules .
 *
 *  \include xwr68xx_mmw_demo_dss_mem_analysis.txt
 *
 *  @section error Note on Error Codes
 *
 *    ------------------------------
 *    When demo runs into error conditions, an error code will be generated and printed out.
 *    Error code is defined as a negative interger. It comes from the following categories:
 *    - Drivers
 *    - Control modules
 *    - Data Processing Chain
 *    - Data Processing Unit
 *    - Demo
 *
 *    The error code is defined as (Module  error code base - Module specific error code).\n
 *    The base error code for the above modules can be found in @ref mmwave_error.h\n
 *    The base error code for DPC and DPU can be found in @ref dp_error.h
 *
 *    Module specific error code is specified in the module's header file.
 *    Examples: 
 *       - UART driver error code is defined in uart.h
 *       - DPC error code is defined in the dpc used in demo (ti/datapath/dpc/objectdetection/objdethwa)
 *
 *  @subsection mmwave_error mmWave module Error Code
 *    Error code from mmWave module is encoded in the following manner:
 *
 *    Bits(31::16)  |  Bits(15::2)   | Bits (1::0)
 *    :-------------|:----------------|:-------------:
 *    mmwave  error  | Subsystem error   | error level
 *
 *    - mmwave error is defined in mmwave.h \n
 *    - Subsystem error is returned from sub-system such as mmwavelink and mailbox driver. \n
 *    - Error level is referred as WARNING leve and ERROR level.
 *    - mmWave exposes an API - MMWave_decodeError() that can be used in demo to decode error code
 *
 *  @subsection mmwave_error_example Example
 *    - Here is an example on how to parse the error code - "-40111"\n
 *        -# The error code is from module with error base "-40000", which indicates it is DPC error.\n
 *        -# By referring to @ref dp_error.h, base "-40100" is from HWA based objectdetection DPC.\n
 *        -# Then find the error code in objectdetection.h for error(-11) as DPC_OBJECTDETECTION_ENOMEM__L3_RAM_RADAR_CUBE
 *
 *    - Another example is with mmWave control module: - "mmWave Config failed [Error code: -3109 Subsystem: 71]"\n
 *        -# The above message indicates the error is from module(-3100 ->mmwave) with error -9(MMWAVE_ECHIRPCFG)\n
 *        -# The subsystem(mmwavelink) error is 71(RL_RET_CODE_CHIRP_TX_ENA_1INVAL_IN) which can be found in mmwavelink.h
 *
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>


/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/family/arm/v7a/Pmu.h>
#include <ti/sysbios/family/arm/v7r/vim/Hwi.h>
#include <ti/sysbios/utils/Load.h>


/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/common/mmwave_sdk_version.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/gpio/gpio.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/drivers/pinmux/pinmux.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/control/dpm/dpm.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/uart/UART.h>
#include <ti/utils/cli/cli.h>
#include <ti/utils/mathutils/mathutils.h>

/* Demo Include Files */
#include <inc/ranging_config.h>
#include <inc/ranging_monitor.h>
#include <inc/ranging_output.h>
#include <Ranging_3_6843_isk_dss/inc/ranging_res.h>
#include <inc/ranging_mss.h>
#include <inc/ranging_rfparser.h>
#include <inc/ranging_adcconfig.h>
#include <ti/demo/utils/mmwdemo_flash.h>

/* Profiler Include Files */
#include <ti/utils/cycleprofiler/cycle_profiler.h>

#ifdef SYS_COMMON_XWR68XX_LOW_POWER_MODE_EN
/* Low Power Library Functions*/
#include <ti/utils/libsleep/libsleep_xwr68xx.h>
#endif

/**
 * @brief Task Priority settings:
 * Mmwave task is at higher priority because of potential async messages from BSS
 * that need quick action in real-time.
 *
 * CLI task must be at a lower priority than object detection
 * dpm task priority because the dynamic CLI command handling in the objection detection
 * dpm task assumes CLI task is held back during this processing. The alternative
 * is to use a semaphore between the two tasks.
 */
#define MMWDEMO_CLI_TASK_PRIORITY                 3
#define MMWDEMO_DPC_OBJDET_DPM_TASK_PRIORITY      4
#define MMWDEMO_MMWAVE_CTRL_TASK_PRIORITY         5

#if (MMWDEMO_CLI_TASK_PRIORITY >= MMWDEMO_DPC_OBJDET_DPM_TASK_PRIORITY)
#error CLI task priority must be < Object Detection DPM task priority
#endif

#define DPC_OBJDET_R4F_INSTANCEID       (0xFEEDFEED)

/* These address offsets are in bytes, when configure address offset in hardware,
   these values will be converted to number of 128bits
   Buffer at offset 0x0U is reserved by BSS, hence offset starts from 0x800
 */
#define MMW_DEMO_CQ_SIGIMG_ADDR_OFFSET          0x800U
#define MMW_DEMO_CQ_RXSAT_ADDR_OFFSET           0x1000U

/* CQ data is at 16 bytes alignment for mulitple chirps */
#define MMW_DEMO_CQ_DATA_ALIGNMENT            16U

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
Ranging_MSS_MCB    gMmwMssMCB;

/**
 * @brief
 *  Global Variable for HSRAM buffer used to share results to remote
 */
Ranging_HSRAM gHSRAM;

/*! HSRAM for processing results */
#pragma DATA_SECTION(gHSRAM, ".demoSharedMem");
#pragma DATA_ALIGN(gHSRAM, 4);

 /*! TCMB RAM buffer for object detection DPC */
#define MMWDEMO_OBJDET_LOCALRAM_SIZE (4U * 1024U)
uint8_t gDPCTCM[MMWDEMO_OBJDET_LOCALRAM_SIZE];
#pragma DATA_SECTION(gDPCTCM, ".dpcLocalRam");
#pragma DATA_ALIGN(gDPCTCM, 4);

/*! L3 RAM buffer for object detection DPC */
uint8_t gMmwL3[SOC_L3RAM_SIZE];
#pragma DATA_SECTION(gMmwL3, ".l3ram");


/**
 * @brief
 *  Global Variable for LDO BYPASS config, PLease consult your 
 * board/EVM user guide before changing the values here
 */
rlRfLdoBypassCfg_t gRFLdoBypassCfg =
{
    .ldoBypassEnable   = 0, /* 1.0V RF supply 1 and 1.0V RF supply 2 */
    .supplyMonIrDrop   = 0, /* IR drop of 3% */
    .ioSupplyIndicator = 0, /* 3.3 V IO supply */
};
/* Calibration Data Save/Restore defines */
#define MMWDEMO_CALIB_FLASH_SIZE	              4096
#define MMWDEMO_CALIB_STORE_MAGIC            (0x7CB28DF9U)

Ranging_calibData gCalibDataStorage;
#pragma DATA_ALIGN(gCalibDataStorage, 8);

/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/

extern void Ranging_CLIInit(uint8_t taskPriority);
extern DPM_ProcChainCfg gDPC_ObjDetRangeDSPCfg;

/**************************************************************************
 ************************* Millimeter Wave Demo Functions prototype *************
 **************************************************************************/

/* MMW demo functions for datapath operation */
static void Ranging_dataPathOpen(void);
static int32_t Ranging_dataPathConfig (void);
static void Ranging_dataPathStart (void);
static void Ranging_dataPathStop (void);
static void Ranging_handleObjectDetResult(DPM_Buffer  *ptrResult);
static void Ranging_DPC_reportFxn
(
    DPM_Report  reportType,
    uint32_t    instanceId,
    int32_t     errCode,
    uint32_t    arg0,
    uint32_t    arg1
);
static void Ranging_transmitProcessedOutput
(
    UART_Handle     uartHandle,
    DPC_Ranging_ExecuteResult   *result,
    Ranging_output_message_stats        *timingInfo
);
static void Ranging_measurementResultOutput(uint8_t placeholder);
static int32_t Ranging_DPM_ioctl_blocking
(
    DPM_Handle handle,
    uint32_t cmd,
    void* arg,
    uint32_t argLen
);

static void Ranging_initTask(UArg arg0, UArg arg1);
static void Ranging_platformInit(Ranging_platformCfg *config);

/* Mmwave control functions */
static void Ranging_mmWaveCtrlTask(UArg arg0, UArg arg1);
static int32_t Ranging_mmWaveCtrlStop (void);
static int32_t Ranging_eventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload);

/* external sleep function when in idle (used in .cfg file) */
void Ranging_sleep(void);

/* Edma related functions */
static void Ranging_edmaInit(void);
static void Ranging_edmaOpen(void);
static void Ranging_EDMA_transferControllerErrorCallbackFxn(EDMA_Handle handle,
                EDMA_transferControllerErrorInfo_t *errorInfo);
static void Ranging_EDMA_errorCallbackFxn(EDMA_Handle handle, EDMA_errorInfo_t *errorInfo);

static int32_t Ranging_configCQ(Ranging_SubFrameCfg *subFrameCfg,
                                           uint8_t numChirpsPerChirpEvent,
                                           uint8_t validProfileIdx);

/* Calibration save/restore APIs */
static int32_t Ranging_calibInit(void);
static int32_t Ranging_calibSave(Ranging_calibDataHeader *ptrCalibDataHdr, Ranging_calibData  *ptrCalibrationData);
static int32_t Ranging_calibRestore(Ranging_calibData  *calibrationData);

#ifdef SYS_COMMON_XWR68XX_LOW_POWER_MODE_EN
/*Function for idle power down and up cycle*/
void idle_power_down(IdleModeCfg   idleModeCfg);
void idle_power_cycle(IdleModeCfg   idleModeCfg);
#endif
/**************************************************************************
 ************************* Millimeter Wave Demo Functions **********************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      EDMA driver init
 *
 *  @param[in] obj          Pointer to data path object
 *  @param[in] instance     EDMA instance
 *
 *  @retval
 *      Not Applicable.
 */
static void Ranging_edmaInit(void)
{
    int32_t errorCode;

    errorCode = EDMA_init(MMW_LVDS_STREAM_EDMA_INSTANCE);
    if (errorCode != EDMA_NO_ERROR)
    {
        System_printf ("Debug: EDMA instance %d initialization returned error %d\n", errorCode);
        Ranging_debugAssert (0);
        return;
    }

    memset(&gMmwMssMCB.EDMA_errorInfo, 0, sizeof(gMmwMssMCB.EDMA_errorInfo));
    memset(&gMmwMssMCB.EDMA_transferControllerErrorInfo, 0, sizeof(gMmwMssMCB.EDMA_transferControllerErrorInfo));
}

/**
 *  @b Description
 *  @n
 *      Call back function for EDMA CC (Channel controller) error as per EDMA API.
 *      Declare fatal error if happens, the output errorInfo can be examined if code
 *      gets trapped here.
 */
static void Ranging_EDMA_errorCallbackFxn(EDMA_Handle handle, EDMA_errorInfo_t *errorInfo)
{
    gMmwMssMCB.EDMA_errorInfo = *errorInfo;
    Ranging_debugAssert(0);
}

/**
 *  @b Description
 *  @n
 *      Call back function for EDMA transfer controller error as per EDMA API.
 *      Declare fatal error if happens, the output errorInfo can be examined if code
 *      gets trapped here.
 */
static void Ranging_EDMA_transferControllerErrorCallbackFxn
(
    EDMA_Handle handle,
    EDMA_transferControllerErrorInfo_t *errorInfo
)
{
    gMmwMssMCB.EDMA_transferControllerErrorInfo = *errorInfo;
    Ranging_debugAssert(0);
}

/**
 *  @b Description
 *  @n
 *      Open EDMA driver instance
 *
 *  @param[in] obj           Pointer to data path object
 *  @param[in] instance      EDMA instance
 *
 *  @retval
 *      Not Applicable.
 */
static void Ranging_edmaOpen(void)
{
    int32_t             errCode;
    EDMA_instanceInfo_t edmaInstanceInfo;
    EDMA_errorConfig_t  errorConfig;
    uint8_t             tc;

    gMmwMssMCB.edmaHandle = EDMA_open(MMW_LVDS_STREAM_EDMA_INSTANCE, &errCode, &edmaInstanceInfo);
    gMmwMssMCB.numEdmaEventQueues = edmaInstanceInfo.numEventQueues;

    if (gMmwMssMCB.edmaHandle == NULL)
    {
        System_printf("Error: Unable to open the EDMA Instance err:%d\n",errCode);
        Ranging_debugAssert (0);
        return;
    }

    errorConfig.isConfigAllEventQueues = true;
    errorConfig.isConfigAllTransferControllers = true;
    errorConfig.isEventQueueThresholdingEnabled = true;
    errorConfig.eventQueueThreshold = EDMA_EVENT_QUEUE_THRESHOLD_MAX;
    errorConfig.isEnableAllTransferControllerErrors = true;

    gMmwMssMCB.isPollEdmaError = false;
    if (edmaInstanceInfo.isErrorInterruptConnected == true)
    {
        errorConfig.callbackFxn = Ranging_EDMA_errorCallbackFxn;
    }
    else
    {
        errorConfig.callbackFxn = NULL;
        gMmwMssMCB.isPollEdmaError = true;
    }

    errorConfig.transferControllerCallbackFxn = Ranging_EDMA_transferControllerErrorCallbackFxn;
    gMmwMssMCB.isPollEdmaTransferControllerError = false;

    for(tc = 0; tc < edmaInstanceInfo.numEventQueues; tc++)
    {
        if (edmaInstanceInfo.isTransferControllerErrorInterruptConnected[tc] == false)
        {
            errorConfig.transferControllerCallbackFxn = NULL;
            gMmwMssMCB.isPollEdmaTransferControllerError = true;
            break;
        }
    }

    if ((errCode = EDMA_configErrorMonitoring(gMmwMssMCB.edmaHandle, &errorConfig)) != EDMA_NO_ERROR)
    {
        //System_printf("Error: EDMA_configErrorMonitoring() failed with errorCode = %d\n", errCode);
        Ranging_debugAssert (0);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      Close EDMA driver instance
 *
 *  @param[in] obj      Pointer to data path object
 *
 *  @retval
 *      Not Applicable.
 */
void Ranging_edmaClose(void)
{
    EDMA_close(gMmwMssMCB.edmaHandle);
}

/**
 *  @b Description
 *  @n
 *      Send assert information through CLI.
 *
 *  @param[in] expression           Expression for evaluation
 *  @param[in] file                 C file that caused assertion
 *  @param[in] line                 Line number in C fine that caused assertion
 *
 */
void _Ranging_debugAssert(int32_t expression, const char *file, int32_t line)
{
    if (!expression) {
        CLI_write ("Exception: %s, line %d.\n",file,line);
    }
}

/**
 *  @b Description
 *  @n
 *      Utility function to set the pending state of configuration.
 *
 *  @param[in] subFrameCfg Pointer to Sub-frame specific configuration
 *  @param[in] offset       Configuration structure offset that uniquely identifies the
 *                          configuration to set to the pending state.
 *
 *  @retval None
 */
static void Ranging_setSubFramePendingState(Ranging_SubFrameCfg *subFrameCfg, uint32_t offset)
{
    switch (offset)
    {
        case MMWDEMO_GUIMONSEL_OFFSET:
            break;
        case MMWDEMO_ADCBUFCFG_OFFSET:
            subFrameCfg->isAdcBufCfgPending = 1;
            break;
        case MMWDEMO_BPMCFG_OFFSET:
            subFrameCfg->isBpmCfgPending = 1;
            break;
        case MMWDEMO_LVDSSTREAMCFG_OFFSET:
            subFrameCfg->isLvdsStreamCfgPending = 1;
            break;
        default:
            Ranging_debugAssert(0);
            break;
    }
}

/**
 *  @b Description
 *  @n
 *      Utility function to find out if all dynamic common configuration is in non-pending (cleared)
 *      state.
 *
 *  @param[in] cfg      Pointer to common specific configuration
 *
 *  @retval 1 if all common configuration is in non-pending state, else return 0
 */
static uint8_t Ranging_isDynObjDetCommonCfgInNonPendingState(Ranging_DPC_ObjDet_CommonCfg *cfg)
{
    uint8_t retVal;

    retVal = (cfg->isCompRxChannelBiasCfgPending    == 0) &&
             (cfg->isMeasureRxChannelBiasCfgPending == 0);

    return(retVal);
}

/**
 *  @b Description
 *  @n
 *      Resets (clears) all pending dynamic common configuration of Object Detection DPC
 *
 *  @param[in] cfg      Object Detection DPC common configuration
 *
 *  @retval
 *      Not Applicable.
 */
static void Ranging_resetDynObjDetCommonCfgPendingState(Ranging_DPC_ObjDet_CommonCfg *cfg)
{
    cfg->isCompRxChannelBiasCfgPending = 0;
    cfg->isMeasureRxChannelBiasCfgPending = 0;
}

/**
 *  @b Description
 *  @n
 *      Resets (clears) all pending static (non-dynamic) configuration
 *
 */
void Ranging_resetStaticCfgPendingState(void)
{
    uint8_t indx;

    for(indx = 0; indx < gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames; indx++)
    {
        gMmwMssMCB.subFrameCfg[indx].isAdcBufCfgPending = 0;
        gMmwMssMCB.subFrameCfg[indx].isBpmCfgPending = 0;
        gMmwMssMCB.subFrameCfg[indx].isLvdsStreamCfgPending = 0;
    }

    gMmwMssMCB.isAnaMonCfgPending = 0;
}

/**
 *  @b Description
 *  @n
 *      Utility function to find out if all configuration (common and sub-frame
 *      specific dynamic config) is in pending state.
 *
 *  @retval 1 if all configuration (common and sub-frame specific dynamic config)
 *            is in pending state, else return 0
 */
uint8_t Ranging_isAllCfgInPendingState(void)
{
    uint8_t indx, flag = 1;

    for(indx = 0; indx < gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames; indx++)
    {
        flag = flag && (gMmwMssMCB.subFrameCfg[indx].isAdcBufCfgPending == 1);
        flag = flag && (gMmwMssMCB.subFrameCfg[indx].isBpmCfgPending == 1);
        flag = flag && (gMmwMssMCB.subFrameCfg[indx].isLvdsStreamCfgPending == 1);
    }

    flag = flag && (gMmwMssMCB.isAnaMonCfgPending == 1);
    return(flag);
}

/**
 *  @b Description
 *  @n
 *      Utility function to find out if all configuration (common and sub-frame
 *      specific dynamic config) is in non-pending (cleared) state.
 *
 *  @retval 1 if all configuration (common and sub-frame specific dynamic config)
 *            is in non-pending state, else return 0
 */
uint8_t Ranging_isAllCfgInNonPendingState(void)
{
    uint8_t indx, flag = 1;

    for(indx = 0; indx < gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames; indx++)
    {
        flag = flag && (gMmwMssMCB.subFrameCfg[indx].isAdcBufCfgPending == 0);
        flag = flag && (gMmwMssMCB.subFrameCfg[indx].isBpmCfgPending == 0);
        flag = flag && (gMmwMssMCB.subFrameCfg[indx].isLvdsStreamCfgPending == 0);
    }

    flag = flag && (Ranging_isDynObjDetCommonCfgInNonPendingState(&gMmwMssMCB.objDetCommonCfg) && flag);
    flag = flag && (gMmwMssMCB.isAnaMonCfgPending == 0);
    return(flag);
}

/**
 *  @b Description
 *  @n
 *      Utility function to apply configuration to specified sub-frame
 *
 *  @param[in] srcPtr Pointer to configuration
 *  @param[in] offset Offset of configuration within the parent structure
 *  @param[in] size   Size of configuration
 *  @param[in] subFrameNum Sub-frame Number (0 based) to apply to, broadcast to
 *                         all sub-frames if special code MMWDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
void Ranging_CfgUpdate(void *srcPtr, uint32_t offset, uint32_t size, int8_t subFrameNum)
{    
    /* if subFrameNum undefined, broadcast to all sub-frames */
    if(subFrameNum == MMWDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG)
    {
        uint8_t  indx;
        for(indx = 0; indx < RL_MAX_SUBFRAMES; indx++)
        {
            memcpy((void *)((uint32_t) &gMmwMssMCB.subFrameCfg[indx] + offset), srcPtr, size);
            Ranging_setSubFramePendingState(&gMmwMssMCB.subFrameCfg[indx], offset);
        }
    }
    else
    {
        /* Apply configuration to specific subframe (or to position zero for the legacy case
           where there is no advanced frame config) */
        memcpy((void *)((uint32_t) &gMmwMssMCB.subFrameCfg[subFrameNum] + offset), srcPtr, size);
        Ranging_setSubFramePendingState(&gMmwMssMCB.subFrameCfg[subFrameNum], offset);
    }
}

/**
 *  @b Description
 *  @n
 *      Utility function to get temperature report from front end and
 *      save it in global structure.
 *
 *  @retval None
 */
void Ranging_getTemperatureReport()
{
    /* Get Temerature report */
    gMmwMssMCB.temperatureStats.tempReportValid = rlRfGetTemperatureReport(RL_DEVICE_MAP_INTERNAL_BSS, 
                        (rlRfTempData_t*)&gMmwMssMCB.temperatureStats.temperatureReport);

}


/**************************************************************************
 ******************** Millimeter Wave Demo Results Transmit Functions *************
 **************************************************************************/
static void Ranging_measurementResultOutput(uint8_t placeholder)
{
    /* Send the received DSS calibration info through CLI */
    CLI_write ("compRangeBiasAndRxChanPhase");
    CLI_write (" %.7f", placeholder);
    int32_t i;
    for (i = 0; i < SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL; i++)
    {
        CLI_write (" %.5f", (float) placeholder);
        CLI_write (" %.5f", (float) placeholder);
    }
    CLI_write ("\n");
}

/** @brief Transmits detection data over UART
*
*    The following data is transmitted:
*    1. Header (size = 32bytes), including "Magic word", (size = 8 bytes)
*       and including the number of TLV items
*    TLV Items:
*    2. If detectedObjects flag is 1 or 2, DPIF_PointCloudCartesian structure containing
*       X,Y,Z location and velocity for detected objects,
*       size = sizeof(DPIF_PointCloudCartesian) * number of detected objects
*    3. If detectedObjects flag is 1, DPIF_PointCloudSideInfo structure containing SNR
*       and noise for detected objects,
*       size = sizeof(DPIF_PointCloudCartesian) * number of detected objects
*    4. If logMagRange flag is set,  rangeProfile,
*       size = number of range bins * sizeof(uint16_t)
*    5. If noiseProfile flag is set,  noiseProfile,
*       size = number of range bins * sizeof(uint16_t)
*    6. If rangeAzimuthHeatMap flag is set, the zero Doppler column of the
*       range cubed matrix, size = number of Rx Azimuth virtual antennas *
*       number of chirps per frame * sizeof(uint32_t)
*    7. If rangeDopplerHeatMap flag is set, the log magnitude range-Doppler matrix,
*       size = number of range bins * number of Doppler bins * sizeof(uint16_t)
*    8. If statsInfo flag is set, the stats information
*   @param[in] uartHandle   UART driver handle
*   @param[in] result       Pointer to result from object detection DPC processing
*   @param[in] timingInfo   Pointer to timing information provided from core that runs data path
*/
static void Ranging_transmitProcessedOutput
(
    UART_Handle     uartHandle,
    DPC_Ranging_ExecuteResult   *result,
    Ranging_output_message_stats        *timingInfo
)
{
    Ranging_output_message_header header;
    Ranging_GuiMonSel   *pGuiMonSel;
    Ranging_SubFrameCfg *subFrameCfg;
    uint32_t tlvIdx = 0;
    uint32_t numPaddingBytes;
    uint32_t packetLen;
    uint8_t padding[RANGING_OUTPUT_MSG_SEGMENT_LEN];
    Ranging_output_message_tl   tl[MMWDEMO_OUTPUT_MSG_MAX];
    int32_t errCode;
    int16_t index;
    DPC_Ranging_Stats   *stats;
    DPC_Ranging_Data    *rangingData;
    char output_data[20];
    int16_t* pBuffer;
    static int16_t num_received = 0;
    
    /* Get subframe configuration */
    subFrameCfg = &gMmwMssMCB.subFrameCfg[result->subFrameIdx];

    /* Get Gui Monitor configuration */
    pGuiMonSel = &subFrameCfg->guiMonSel;

    /* Clear message header */
    memset((void *)&header, 0, sizeof(Ranging_output_message_header));

    /******************************************************************
       Send out data that is enabled, Since processing results are from DSP,
       address translation is needed for buffer pointers
    *******************************************************************/
    {
        stats = (DPC_Ranging_Stats *) SOC_translateAddress((uint32_t)result->stats,
                                                     SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
                                                     &errCode);
        DebugP_assert ((uint32_t)stats != SOC_TRANSLATEADDR_INVALID);


        result->radarCube.data  = (void *) SOC_translateAddress((uint32_t) result->radarCube.data,
                                                     SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
                                                     &errCode);
        DebugP_assert ((uint32_t) result->radarCube.data != SOC_TRANSLATEADDR_INVALID);

        rangingData = (DPC_Ranging_Data *) SOC_translateAddress((uint32_t) result->rangingData,
                                                     SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
                                                     &errCode);
        DebugP_assert ((uint32_t) rangingData != SOC_TRANSLATEADDR_INVALID);
    }

    // Data is arranged as:

    // ADC Data In - pHwRes->radarCube.data
    // rangingObj->radarCubebuf                = (cmplx16ImRe_t *)pHwRes->radarCube.data;
    // rangingObj->fftOfMagnitude              = (cmplx16ImRe_t *)rangingObj->radarCubebuf                  + pStaticCfg->ADCBufData.dataSize;
    // rangingObj->magnitudeData               = (cmplx16ImRe_t *)rangingObj->fftOfMagnitude                + pStaticCfg->ADCBufData.dataSize;
    // rangingObj->vectorMultiplyOfFFtedData   = (cmplx16ImRe_t *)rangingObj->magnitudeData                 + pStaticCfg->ADCBufData.dataSize;
    // rangingObj->iFftData                    = (cmplx16ImRe_t *)rangingObj->vectorMultiplyOfFFtedData     + pStaticCfg->ADCBufData.dataSize;

    // ADC Data
    if(rangingData->wasCodeDetected)
    {
        snprintf(output_data,
                 sizeof(output_data),
                 "\r\nCODE DETECT\r\n");

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));
    }
    if(num_received >= 100)
    {
        Task_sleep(1);         // 1 millisecond
        snprintf(output_data,
                 sizeof(output_data),
                 "\r\nEarly: %d\r\n",
                 rangingData->earlyValue);

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        snprintf(output_data,
                 sizeof(output_data),
                 "Prompt: %d\r\n",
                 rangingData->promptValue);

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        snprintf(output_data,
                 sizeof(output_data),
                 "Late: %d\r\n",
                 rangingData->lateValue);

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        snprintf(output_data,
                 sizeof(output_data),
                 "Index: %d\r\n",
                 rangingData->promptIndex);

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        snprintf(output_data,
                 sizeof(output_data),
                 "\r\nadc_data\r\n");

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        pBuffer = (int16_t *)result->radarCube.data;
        for(index = 0; index < gMmwMssMCB.subFrameCfg[result->subFrameIdx].numAdcSamples; index++)
        {
            snprintf(output_data,
                     sizeof(output_data),
                     ",%d,%d",
                     pBuffer[2*index],
                     pBuffer[2*index+1]);

            UART_writePolling (uartHandle,
                               (uint8_t*)&output_data,
                               strlen(output_data));

            Task_sleep(1);         // 1 millisecond
        }

        // magnitudeData
        snprintf(output_data,
                 sizeof(output_data),
                 "\r\nmagnitude_data\r\n");

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        pBuffer = (int16_t *)result->radarCube.data + 2*gMmwMssMCB.subFrameCfg[result->subFrameIdx].numAdcSamples;
        for(index = 0; index < gMmwMssMCB.subFrameCfg[result->subFrameIdx].numAdcSamples; index++)
        {
            snprintf(output_data,
                     sizeof(output_data),
                     ",%d,%d",
                     pBuffer[2*index + 0],
                     pBuffer[2*index + 1]);

            UART_writePolling (uartHandle,
                               (uint8_t*)&output_data,
                               strlen(output_data));

            Task_sleep(1);         // 1 millisecond
        }

        // fftOfMagnitude
        snprintf(output_data,
                 sizeof(output_data),
                 "\r\n---------------\r\n");

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));
        snprintf(output_data,
                 sizeof(output_data),
                 "\r\nfft_of_mag\r\n");

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        pBuffer = (int16_t *)result->radarCube.data + 4*gMmwMssMCB.subFrameCfg[result->subFrameIdx].numAdcSamples;
        for(index = 0; index < gMmwMssMCB.subFrameCfg[result->subFrameIdx].numAdcSamples; index++)
        {
            snprintf(output_data,
                     sizeof(output_data),
                     ",%d,%d",
                     pBuffer[2*index + 0],
                     pBuffer[2*index + 1]);

            UART_writePolling (uartHandle,
                               (uint8_t*)&output_data,
                               strlen(output_data));

            Task_sleep(1);         // 1 millisecond
        }

        // vectorMultiplyOfFFtedData - int 32s
        snprintf(output_data,
                 sizeof(output_data),
                 "\r\nvector_multiply\r\n");

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        pBuffer = (int16_t *)result->radarCube.data + 6*gMmwMssMCB.subFrameCfg[result->subFrameIdx].numAdcSamples;
        for(index = 0; index < gMmwMssMCB.subFrameCfg[result->subFrameIdx].numAdcSamples; index++)
        {
            snprintf(output_data,
                     sizeof(output_data),
                     ",%d,%d",
                     ((cmplx32ImRe_t *)pBuffer)[index].imag,
                     ((cmplx32ImRe_t *)pBuffer)[index].real);

            UART_writePolling (uartHandle,
                               (uint8_t*)&output_data,
                               strlen(output_data));

            Task_sleep(1);         // 1 millisecond
        }

        // IFFT of two vectors multiplied together - int 32s
        snprintf(output_data,
                 sizeof(output_data),
                 "\r\nifft_data\r\n");

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        pBuffer = (int16_t *)result->radarCube.data + 10*gMmwMssMCB.subFrameCfg[result->subFrameIdx].numAdcSamples;
        for(index = 0; index < gMmwMssMCB.subFrameCfg[result->subFrameIdx].numAdcSamples; index++)
        {
            snprintf(output_data,
                     sizeof(output_data),
                     ",%d,%d",
                     ((int32_t *)pBuffer)[2*index + 0],
                     ((int32_t *)pBuffer)[2*index + 1]);

            UART_writePolling (uartHandle,
                               (uint8_t*)&output_data,
                               strlen(output_data));

            Task_sleep(1);         // 1 millisecond
        }

        // Magnitude of the IFFT of two vectors multiplied together - int 32s
        snprintf(output_data,
                 sizeof(output_data),
                 "\r\nmag_ifft\r\n");

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        pBuffer = (int16_t *)result->radarCube.data + 14*gMmwMssMCB.subFrameCfg[result->subFrameIdx].numAdcSamples;
        for(index = 0; index < gMmwMssMCB.subFrameCfg[result->subFrameIdx].numAdcSamples; index++)
        {
            snprintf(output_data,
                     sizeof(output_data),
                     ",%d,%d",
                     ((cmplx32ImRe_t *)pBuffer)[index].imag,
                     ((cmplx32ImRe_t *)pBuffer)[index].real);

            UART_writePolling (uartHandle,
                               (uint8_t*)&output_data,
                               strlen(output_data));

            Task_sleep(1);         // 1 millisecond
        }

        // Complex conjugate of the FFT'd gold code
        snprintf(output_data,
                 sizeof(output_data),
                 "\r\ncmplx_conj_gold\r\n");

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        pBuffer = (int16_t *)result->radarCube.data + 18*gMmwMssMCB.subFrameCfg[result->subFrameIdx].numAdcSamples;
        for(index = 0; index < gMmwMssMCB.subFrameCfg[result->subFrameIdx].numAdcSamples; index++)
        {
            snprintf(output_data,
                     sizeof(output_data),
                     ",%d,%d",
                     pBuffer[2*index + 0],
                     pBuffer[2*index + 1]);

            UART_writePolling (uartHandle,
                               (uint8_t*)&output_data,
                               strlen(output_data));

            Task_sleep(1);         // 1 millisecond
        }
    }
    /**/

    // Header:
    header.platform =  0xA6843;
    header.magicWord[0] = 0x0102;
    header.magicWord[1] = 0x0304;
    header.magicWord[2] = 0x0506;
    header.magicWord[3] = 0x0708;
    header.version =    MMWAVE_SDK_VERSION_BUILD |
                        (MMWAVE_SDK_VERSION_BUGFIX << 8) |
                        (MMWAVE_SDK_VERSION_MINOR << 16) |
                        (MMWAVE_SDK_VERSION_MAJOR << 24);

    packetLen = sizeof(Ranging_output_message_header);
    if (pGuiMonSel->noiseProfile)
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_NOISE_PROFILE;
        //tl[tlvIdx].length = sizeof(uint16_t) * subFrameCfg->numRangeBins;
        tl[tlvIdx].length = sizeof(uint16_t) * subFrameCfg->numAdcSamples;
        packetLen += sizeof(Ranging_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    if (pGuiMonSel->statsInfo)
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_STATS;
        tl[tlvIdx].length = sizeof(Ranging_output_message_stats);
        packetLen += sizeof(Ranging_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;

        Ranging_getTemperatureReport();
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_TEMPERATURE_STATS;
        tl[tlvIdx].length = sizeof(Ranging_temperatureStats);
        packetLen += sizeof(Ranging_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }

    header.numTLVs = tlvIdx;
    /* Round up packet length to multiple of RANGING_OUTPUT_MSG_SEGMENT_LEN */
    header.totalPacketLen = RANGING_OUTPUT_MSG_SEGMENT_LEN *
            ((packetLen + (RANGING_OUTPUT_MSG_SEGMENT_LEN-1))/RANGING_OUTPUT_MSG_SEGMENT_LEN);
    header.timeCpuCycles = Pmu_getCount(0);
    header.frameNumber = stats->frameStartIntCounter;
    header.subFrameNumber = result->subFrameIdx;

    UART_writePolling (uartHandle,
                       (uint8_t*)&header,
                       sizeof(Ranging_output_message_header));

    tlvIdx = 0;

    /* Send noise profile */
    if (pGuiMonSel->noiseProfile)
    {
        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(Ranging_output_message_tl));
        tlvIdx++;
    }

    /* Send stats information */
    if (pGuiMonSel->statsInfo == 1)
    {
        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(Ranging_output_message_tl));

        /* Address translation is done when buffer is received*/
        UART_writePolling (uartHandle,
                           (uint8_t*)timingInfo,
                           tl[tlvIdx].length);
        tlvIdx++;        
        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(Ranging_output_message_tl));
        UART_writePolling (uartHandle,
                           (uint8_t*)&gMmwMssMCB.temperatureStats,
                           tl[tlvIdx].length);
        tlvIdx++;
    }

    /* Send padding bytes */
    numPaddingBytes = RANGING_OUTPUT_MSG_SEGMENT_LEN - (packetLen & (RANGING_OUTPUT_MSG_SEGMENT_LEN-1));
    if (numPaddingBytes<RANGING_OUTPUT_MSG_SEGMENT_LEN)
    {
        UART_writePolling (uartHandle,
                            (uint8_t*)padding,
                            numPaddingBytes);
    }
    num_received++;
}

/**************************************************************************
 ******************** Millimeter Wave Demo control path Functions *****************
 **************************************************************************/
/**
 *  @b Description
 *  @n
 *      The function is used to trigger the Front end to stop generating chirps.
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t Ranging_mmWaveCtrlStop (void)
{
    int32_t                 errCode = 0;

    DebugP_log0("App: Issuing MMWave_stop\n");

    /* Stop the mmWave module: */
    if (MMWave_stop (gMmwMssMCB.ctrlHandle, &errCode) < 0)
    {
        MMWave_ErrorLevel   errorLevel;
        int16_t             mmWaveErrorCode;
        int16_t             subsysErrorCode;

        /* Error/Warning: Unable to stop the mmWave module */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
        if (errorLevel == MMWave_ErrorLevel_ERROR)
        {
            /* Error: Display the error message: */
            System_printf ("Error: mmWave Stop failed [Error code: %d Subsystem: %d]\n",
                            mmWaveErrorCode, subsysErrorCode);

            /* Not expected */
            Ranging_debugAssert(0);
        }
        else
        {
            /* Warning: This is treated as a successful stop. */
            System_printf ("mmWave Stop error ignored [Error code: %d Subsystem: %d]\n",
                            mmWaveErrorCode, subsysErrorCode);
        }
    }

    return errCode;
}

/**
 *  @b Description
 *  @n
 *      The task is used to provide an execution context for the mmWave
 *      control task
 *
 *  @retval
 *      Not Applicable.
 */
static void Ranging_mmWaveCtrlTask(UArg arg0, UArg arg1)
{
    int32_t errCode;

    while (1)
    {
        /* Execute the mmWave control module: */
        if (MMWave_execute (gMmwMssMCB.ctrlHandle, &errCode) < 0)
        {
            //System_printf ("Error: mmWave control execution failed [Error code %d]\n", errCode);
            Ranging_debugAssert (0);
        }
    }
}

/**************************************************************************
 ******************** Millimeter Wave Demo data path Functions *******************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Help function to make DPM_ioctl blocking until response is reported
 *
 *  @retval
 *      Success         -0
 *      Failed          <0
 */
static int32_t Ranging_DPM_ioctl_blocking
(
    DPM_Handle handle,
    uint32_t cmd,
    void* arg,
    uint32_t argLen
)
{
    int32_t retVal = 0;

    retVal = DPM_ioctl(handle,
                     cmd,
                     arg,
                     argLen);

    if(retVal == 0)
    {
        /* Wait until ioctl completed */
        Semaphore_pend(gMmwMssMCB.DPMioctlSemHandle, BIOS_WAIT_FOREVER);
    }

    return(retVal);
}

/**
 *  @b Description
 *  @n
 *    Function that configures the BPM chirps based on the stored BPM CLI commands.
 *    The MMW demo supports only a fixed BPM scheme and this scheme is implemented
 *    by this function.
 *
 *  @retval
 *      0  - Success.
 *      <0 - Failed with errors
 */
int32_t Ranging_bpmConfig(void)
{
    uint8_t                subframe;
    uint8_t                numberOfSubframes;
    int32_t                errCode;
    rlBpmChirpCfg_t        bpmChirpCfg;
    MMWave_BpmChirpHandle  bpmChirpHandle;

    /* BPM configuration for all valid subframes */
    numberOfSubframes = Ranging_RFParser_getNumSubFrames(&gMmwMssMCB.cfg.ctrlCfg);

    /*    Note: The only BPM scheme supported by AoA DPU uses both Azimuth TX antennas
     *    (only 2 TX is allowed and the 2 TX must be the 2 azimuth antennas).
     *    If 3TX antennas are enabled in a given profile ,BPM can not be enabled.
     *
     *    Based on 68xx device antenna pattern layout, BPM can be enabled when only TX1 and TX3 are enabled.
     */
    for(subframe = 0; subframe < numberOfSubframes; subframe++)
    {
        /* Is BPM enabled*/
        if(gMmwMssMCB.subFrameCfg[subframe].bpmCfg.isEnabled)
        {
            /* Configure chirp0 (++) */
            memset ((void *)&bpmChirpCfg, 0, sizeof(rlBpmChirpCfg_t));
            bpmChirpCfg.chirpStartIdx = gMmwMssMCB.subFrameCfg[subframe].bpmCfg.chirp0Idx;
            bpmChirpCfg.chirpEndIdx   = gMmwMssMCB.subFrameCfg[subframe].bpmCfg.chirp0Idx;
            /* Phase configuration: Both TX antenna should be positive (for 68xx device, they are TX1 and TX3 ) 
              Note: rlBpmChirpCfg_t uses TX0 - TX2 which refers to TX1 - TX3 on 68xx devices
             */
            bpmChirpCfg.constBpmVal = 0U;
                        
            bpmChirpHandle = MMWave_addBpmChirp (gMmwMssMCB.ctrlHandle, &bpmChirpCfg, &errCode);
            if (bpmChirpHandle == NULL)
            {
                System_printf ("Error: Unable to add BPM cfg chirp 0. Subframe %d [Error code %d]\n",subframe, errCode);
                return -1;
            }

            /*Configure chirp1 (+-) */
            memset ((void *)&bpmChirpCfg, 0, sizeof(rlBpmChirpCfg_t));
            bpmChirpCfg.chirpStartIdx = gMmwMssMCB.subFrameCfg[subframe].bpmCfg.chirp1Idx;
            bpmChirpCfg.chirpEndIdx   = gMmwMssMCB.subFrameCfg[subframe].bpmCfg.chirp1Idx;
            /* Phase configuration:, first Azimuth Tx antenna should be positive(for 68xx device, this is TX1)
                                 second Azimuth Tx antenna should be negtive(for 68xx device, this is TX3) */
            bpmChirpCfg.constBpmVal = 0x30U;
            
            bpmChirpHandle = MMWave_addBpmChirp (gMmwMssMCB.ctrlHandle, &bpmChirpCfg, &errCode);
            if (bpmChirpHandle == NULL)
            {
                System_printf ("Error: Unable to add BPM cfg chirp 1. Subframe %d [Error code %d]\n",subframe, errCode);
                return -1;
            }
        }
        else
        {
            /*BPM is disabled.
              Configure the range of chirps [chirp0Idx..chirp1Idx]
              all to have zero phase.*/
            memset ((void *)&bpmChirpCfg, 0, sizeof(rlBpmChirpCfg_t));
            bpmChirpCfg.chirpStartIdx = gMmwMssMCB.subFrameCfg[subframe].bpmCfg.chirp0Idx;
            bpmChirpCfg.chirpEndIdx   = gMmwMssMCB.subFrameCfg[subframe].bpmCfg.chirp1Idx;
            bpmChirpCfg.constBpmVal = 0U;

            bpmChirpHandle = MMWave_addBpmChirp (gMmwMssMCB.ctrlHandle, &bpmChirpCfg, &errCode);
            if (bpmChirpHandle == NULL)
            {
                System_printf ("Error: Unable to add BPM cfg for BPM disabled. Subframe %d [Error code %d]\n",subframe, errCode);
                return -1;
            }
        }
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Perform Data path driver open 
 *
 *  @retval
 *      Not Applicable.
 */
static void Ranging_dataPathOpen(void)
{
    gMmwMssMCB.adcBufHandle = Ranging_ADCBufOpen(gMmwMssMCB.socHandle);
    if(gMmwMssMCB.adcBufHandle == NULL)
    {
        Ranging_debugAssert(0);
    }
}

/**
 *  @b Description
 *  @n
 *      Function to configure CQ.
 *
 *  @param[in] subFrameCfg Pointer to sub-frame config
 *  @param[in] numChirpsPerChirpEvent number of chirps per chirp event
 *  @param[in] validProfileIdx valid profile index
 *
 *  @retval
 *      0 if no error, else error (there will be system prints for these).
 */
static int32_t Ranging_configCQ(Ranging_SubFrameCfg *subFrameCfg,
                                uint8_t numChirpsPerChirpEvent,
                                uint8_t validProfileIdx)
{
    Ranging_AnaMonitorCfg*      ptrAnaMonitorCfg;
    ADCBuf_CQConf               cqConfig;
    rlRxSatMonConf_t*           ptrSatMonCfg;
    rlSigImgMonConf_t*          ptrSigImgMonCfg;
    int32_t                     retVal;
    uint16_t                    cqChirpSize;

    /* Get analog monitor configuration */
    ptrAnaMonitorCfg = &gMmwMssMCB.anaMonCfg;

    /* Config mmwaveLink to enable Saturation monitor - CQ2 */
    ptrSatMonCfg = &gMmwMssMCB.cqSatMonCfg[validProfileIdx];

    if (ptrAnaMonitorCfg->rxSatMonEn)
    {
        if (ptrSatMonCfg->profileIndx != validProfileIdx)
        {
            System_printf ("Error: Saturation monitoring (globally) enabled but not configured for profile(%d)\n",
                           validProfileIdx);
            Ranging_debugAssert(0);
        }

        retVal = ranging_cfgRxSaturationMonitor(ptrSatMonCfg);
        if(retVal != 0)
        {
            System_printf ("Error: rlRfRxIfSatMonConfig returns error = %d for profile(%d)\n",
                           retVal, ptrSatMonCfg->profileIndx);
            goto exit;
        }
    }

    /* Config mmwaveLink to enable Saturation monitor - CQ1 */
    ptrSigImgMonCfg = &gMmwMssMCB.cqSigImgMonCfg[validProfileIdx];

    if (ptrAnaMonitorCfg->sigImgMonEn)
    {
        if (ptrSigImgMonCfg->profileIndx != validProfileIdx)
        {
            System_printf ("Error: Sig/Image monitoring (globally) enabled but not configured for profile(%d)\n",
                           validProfileIdx);
            Ranging_debugAssert(0);
        }

        retVal = ranging_cfgRxSigImgMonitor(ptrSigImgMonCfg);
        if(retVal != 0)
        {
            System_printf ("Error: rlRfRxSigImgMonConfig returns error = %d for profile(%d)\n",
                           retVal, ptrSigImgMonCfg->profileIndx);
            goto exit;
        }
    }

    retVal = ranging_cfgAnalogMonitor(ptrAnaMonitorCfg);
    if (retVal != 0)
    {
        System_printf ("Error: rlRfAnaMonConfig returns error = %d\n", retVal);
        goto exit;
    }

    if(ptrAnaMonitorCfg->rxSatMonEn || ptrAnaMonitorCfg->sigImgMonEn)
    {
        /* CQ driver config */
        memset((void *)&cqConfig, 0, sizeof(ADCBuf_CQConf));
        cqConfig.cqDataWidth = 0; /* 16bit for mmw demo */
        cqConfig.cq1AddrOffset = MMW_DEMO_CQ_SIGIMG_ADDR_OFFSET; /* CQ1 starts from the beginning of the buffer */
        cqConfig.cq2AddrOffset = MMW_DEMO_CQ_RXSAT_ADDR_OFFSET;  /* Address should be 16 bytes aligned */

        retVal = ADCBuf_control(gMmwMssMCB.adcBufHandle, ADCBufMMWave_CMD_CONF_CQ, (void *)&cqConfig);
        if (retVal < 0)
        {
            System_printf ("Error: MMWDemoDSS Unable to configure the CQ\n");
            Ranging_debugAssert(0);
        }
    }

    if (ptrAnaMonitorCfg->sigImgMonEn)
    {
        /* This is for 16bit format in mmw demo, signal/image band data has 2 bytes/slice
           For other format, please check DFP interface document
         */
        cqChirpSize = (ptrSigImgMonCfg->numSlices + 1) * sizeof(uint16_t);
        cqChirpSize = MATHUTILS_ROUND_UP_UNSIGNED(cqChirpSize, MMW_DEMO_CQ_DATA_ALIGNMENT);
        subFrameCfg->sigImgMonTotalSize = cqChirpSize * numChirpsPerChirpEvent;
    }

    if (ptrAnaMonitorCfg->rxSatMonEn)
    {
        /* This is for 16bit format in mmw demo, saturation data has one byte/slice
           For other format, please check DFP interface document
         */
        cqChirpSize = (ptrSatMonCfg->numSlices + 1) * sizeof(uint8_t);
        cqChirpSize = MATHUTILS_ROUND_UP_UNSIGNED(cqChirpSize, MMW_DEMO_CQ_DATA_ALIGNMENT);
        subFrameCfg->satMonTotalSize = cqChirpSize * numChirpsPerChirpEvent;
    }

exit:
    return(retVal);
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the data path based on the chirp profile.
 *      After this function is executed, the data path processing will ready to go
 *      when the ADC buffer starts receiving samples corresponding to the chirps.
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t Ranging_dataPathConfig (void)
{
    int32_t                         errCode;
    MMWave_CtrlCfg                  *ptrCtrlCfg;
    Ranging_DPC_ObjDet_CommonCfg *objDetCommonCfg;
    Ranging_SubFrameCfg             *subFrameCfg;
    int8_t                          subFrameIndx;
    Ranging_RFParserOutParams       RFparserOutParams;
    DPC_Ranging_PreStartCfg objDetPreStartR4fCfg;
    DPC_Ranging_StaticCfg *staticCfg;
    DPC_Ranging_PreStartCfg objDetPreStartDspCfg;

    /* Get data path object and control configuration */
    ptrCtrlCfg = &gMmwMssMCB.cfg.ctrlCfg;

    objDetCommonCfg = &gMmwMssMCB.objDetCommonCfg;
    staticCfg = &objDetPreStartR4fCfg.staticCfg;

    /* Get RF frequency scale factor */
    gMmwMssMCB.rfFreqScaleFactor = SOC_getDeviceRFFreqScaleFactor(gMmwMssMCB.socHandle, &errCode);
    if (errCode < 0)
    {
        System_printf ("Error: Unable to get RF scale factor [Error:%d]\n", errCode);
        Ranging_debugAssert(0);
    }

    gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames =
        Ranging_RFParser_getNumSubFrames(ptrCtrlCfg);

    DebugP_log0("App: Issuing Pre-start Common Config IOCTL to R4F\n");

//    /* DPC pre-start common config */
//    preStartCommonCfg.numSubFrames = gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames;
//    errCode = DPM_ioctl (gMmwMssMCB.objDetDpmHandle,
//                         DPC_RANGING_IOCTL__STATIC_PRE_START_COMMON_CFG,
//                         &preStartCommonCfg,
//                         sizeof (preStartCommonCfg));
//    if (errCode < 0)
//    {
//        System_printf ("Error: Unable to send DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_COMMON_CFG [Error:%d]\n", errCode);
//        goto exit;
//    }
    DebugP_log0("App: Issuing Pre-start Common Config IOCTL to DSP\n");

    /* DPC pre-start common config */
    errCode = Ranging_DPM_ioctl_blocking (gMmwMssMCB.objDetDpmHandle,
                         DPC_RANGING_IOCTL__STATIC_PRE_START_COMMON_CFG,
                         &objDetCommonCfg->preStartCommonCfg,
                         sizeof (DPC_Ranging_PreStartCommonCfg));

    if (errCode < 0)
    {
        System_printf ("Error: Unable to send DPC_OBJDET_IOCTL__STATIC_PRE_START_COMMON_CFG [Error:%d]\n", errCode);
        goto exit;
    }

    Ranging_resetDynObjDetCommonCfgPendingState(&gMmwMssMCB.objDetCommonCfg);

    /* Reason for reverse loop is that when sensor is started, the first sub-frame
     * will be active and the ADC configuration needs to be done for that sub-frame
     * before starting (ADC buf hardware does not have notion of sub-frame, it will
     * be reconfigured every sub-frame). This cannot be alternatively done by calling
     * the Ranging_ADCBufConfig function only for the first sub-frame because this is
     * a utility API that computes the rxChanOffset that is part of ADC dataProperty
     * which will be used by range DPU and therefore this computation is required for
     * all sub-frames.
     */
    for(subFrameIndx = gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames -1; subFrameIndx >= 0;
        subFrameIndx--)
    {
        subFrameCfg  = &gMmwMssMCB.subFrameCfg[subFrameIndx];

        /*****************************************************************************
         * Data path :: Algorithm Configuration
         *****************************************************************************/

        /* Parse the profile and chirp configs and get the valid number of TX Antennas */
        errCode = Ranging_RFParser_parseConfig(&RFparserOutParams, subFrameIndx,
                                         &gMmwMssMCB.cfg.openCfg, ptrCtrlCfg,
                                         &subFrameCfg->adcBufCfg,
                                         gMmwMssMCB.rfFreqScaleFactor,
                                         subFrameCfg->bpmCfg.isEnabled); 

        if (errCode != 0)
        {
            System_printf ("Error: Ranging_RFParser_parseConfig [Error:%d]\n", errCode);
            goto exit;
        }

        subFrameCfg->numChirpsPerChirpEvent = RFparserOutParams.numChirpsPerChirpEvent;
        subFrameCfg->adcBufChanDataSize = RFparserOutParams.adcBufChanDataSize;
        subFrameCfg->numAdcSamples = RFparserOutParams.numAdcSamples;
        subFrameCfg->numChirpsPerSubFrame = RFparserOutParams.numChirpsPerFrame;
        subFrameCfg->numVirtualAntennas = RFparserOutParams.numVirtualAntennas;

        errCode = Ranging_ADCBufConfig(gMmwMssMCB.adcBufHandle,
                                 gMmwMssMCB.cfg.openCfg.chCfg.rxChannelEn,
                                 subFrameCfg->numChirpsPerChirpEvent,
                                 subFrameCfg->adcBufChanDataSize,
                                 &subFrameCfg->adcBufCfg,
                                 &staticCfg->ADCBufData.dataProperty.rxChanOffset[0]);
        if (errCode < 0)
        {
            System_printf("Error: ADCBuf config failed with error[%d]\n", errCode);
            Ranging_debugAssert (0);
        }

        errCode = Ranging_configCQ(subFrameCfg, RFparserOutParams.numChirpsPerChirpEvent,
                                   RFparserOutParams.validProfileIdx);

        if (errCode < 0)
        {
            goto exit;
        }

        /* DPC pre-start config */
        {
            int32_t idx;

            objDetPreStartR4fCfg.subFrameNum = subFrameIndx;

            /* Fill static configuration */
            // THIS IS WHERE THE ADC PLACES ITS SAMPLES
            // In the DSS we configure the EDMA to read from here
            staticCfg->ADCBufData.data = (void *)SOC_XWR68XX_MSS_ADCBUF_BASE_ADDRESS;
            // The way this buffer is filled is configured in Ranging_ADCBufConfig, in ranging_adcconfig.c

            // The EDMA is configured to read the ADC buffer in rangingDSP_ConfigDataInEDMA, in rangingdsp.c
            // The EDMA is configured as A Sync events, meaning that every event reads aCount bytes.
            // Successive A Sync events on the same channel increment the source address by srcBIdx = 2 * numAdcSamples
            // So the first PING transfer is antenna zero from location     ADCBUF + 0                  (set in rangingDSP_ConfigDataInEDMA)
            // The first PONG transfer is antenna one from location         ADCBUF + numAdcSamples      (set in rangingDSP_ConfigDataInEDMA)
            // The second  PING transfer is antenna two from location       ADCBUF + 2*numAdcSamples    (set in rangingDSP_ConfigDataInEDMA (base address + srcBIdx))
            // The second  PONG transfer is antenna three from location     ADCBUF + 3*numAdcSamples    (set in rangingDSP_ConfigDataInEDMA (base address + srcBIdx))

            // The destination buffer is configured in DPC_ObjDetDSP_rangeConfig in objectdetection.c
            // It is configured to hold two antennas worth of samples, PING/PONG.
            // It is CoreL1RamObj:
            //      hwRes->adcDataInSize = 2U * staticCfg->ADCBufData.dataProperty.numAdcSamples * sizeof(cmplx16ImRe_t);
            //      hwRes->adcDataIn = (cmplx16ImRe_t *)DPC_ObjDetDSP_MemPoolAlloc(CoreL1RamObj,
            //                              hwRes->adcDataInSize,
            //                              DPU_RANGINGDSP_ADCDATAIN_BYTE_ALIGNMENT_DSP);
            // We actually command the EDMA data transfer in DPU_RangingDSP_process, in the DSS, rangingdsp.c.
            // PING and PONG transfers are started, then PING is processed to an output buffer
            // Then PING2 transfer is started, then PONG is processed, then PONG2 transfer is started, then PING2 is processed, then PONG2 is processed.
            // DPU_RangingDSP_process is called from DPC_Ranging_execute, which is the data processing chain execute function, in objectdetection.c.
            // DPC_Ranging_chirpEvent calls DPM_notifyExecute, which alerts the DPC that it should call the execute function.

            staticCfg->ADCBufData.dataProperty.adcBits = 2; /* 16-bit */

            /* only complex format supported */
            Ranging_debugAssert(subFrameCfg->adcBufCfg.adcFmt == 0);

            staticCfg->ADCBufData.dataProperty.dataFmt = DPIF_DATAFORMAT_COMPLEX16_IMRE;
            staticCfg->ADCBufData.dataProperty.interleave = DPIF_RXCHAN_NON_INTERLEAVE_MODE;
            staticCfg->ADCBufData.dataProperty.numAdcSamples = RFparserOutParams.numAdcSamples;
            staticCfg->ADCBufData.dataProperty.numChirpsPerChirpEvent = RFparserOutParams.numChirpsPerChirpEvent;
            staticCfg->ADCBufData.dataProperty.numRxAntennas = RFparserOutParams.numRxAntennas;
            staticCfg->ADCBufData.dataSize = RFparserOutParams.numRxAntennas * RFparserOutParams.numAdcSamples * sizeof(cmplx16ImRe_t);
            staticCfg->numChirpsPerFrame = RFparserOutParams.numChirpsPerFrame;
            staticCfg->numTxAntennas = RFparserOutParams.numTxAntennas;
            staticCfg->numVirtualAntennas = RFparserOutParams.numVirtualAntennas;
            staticCfg->centerFreq = RFparserOutParams.centerFreq;

            /***********************************************************************
              Pre-start preparation for objdetdsp
             ***********************************************************************/
            /* Reset preStart config */
            memset((void *)&objDetPreStartDspCfg, 0, sizeof(DPC_Ranging_PreStartCfg));

            /* DPC configuration */
            objDetPreStartDspCfg.subFrameNum = subFrameIndx;

            memcpy((void *)&objDetPreStartDspCfg.staticCfg.ADCBufData, 
                    (void *)&staticCfg->ADCBufData,
                    sizeof(DPIF_ADCBufData));
            objDetPreStartDspCfg.staticCfg.isValidProfileHasOneTxPerChirp = RFparserOutParams.validProfileHasOneTxPerChirp;
            objDetPreStartDspCfg.staticCfg.numChirpsPerFrame    = RFparserOutParams.numChirpsPerFrame;
            objDetPreStartDspCfg.staticCfg.numTxAntennas        = RFparserOutParams.numTxAntennas;
            objDetPreStartDspCfg.staticCfg.numVirtualAntAzim    = RFparserOutParams.numVirtualAntAzim;
            objDetPreStartDspCfg.staticCfg.numVirtualAntElev    = RFparserOutParams.numVirtualAntElev;
            objDetPreStartDspCfg.staticCfg.numVirtualAntennas   = RFparserOutParams.numVirtualAntennas;
            objDetPreStartDspCfg.staticCfg.isBpmEnabled         = subFrameCfg->bpmCfg.isEnabled;
            objDetPreStartDspCfg.staticCfg.centerFreq           = RFparserOutParams.centerFreq;
            objDetPreStartDspCfg.staticCfg.adcSampleRate        = RFparserOutParams.adcSampleRate;

            for (idx = 0; idx < RFparserOutParams.numRxAntennas; idx++)
            {
                objDetPreStartDspCfg.staticCfg.rxAntOrder[idx] = RFparserOutParams.rxAntOrder[idx];
            }
            for (idx = 0; idx < RFparserOutParams.numTxAntennas; idx++)
            {
                objDetPreStartDspCfg.staticCfg.txAntOrder[idx] = RFparserOutParams.txAntOrder[idx];
            }

            /* DPC running on remote core, address need to be converted */
            objDetPreStartDspCfg.staticCfg.ADCBufData.data = (void *) SOC_translateAddress((uint32_t)staticCfg->ADCBufData.data,
                                                 SOC_TranslateAddr_Dir_TO_OTHER_CPU,
                                                 &errCode);
            DebugP_assert ((uint32_t)staticCfg->ADCBufData.data != SOC_TRANSLATEADDR_INVALID);

            /* send pre-start config */
            errCode = Ranging_DPM_ioctl_blocking (gMmwMssMCB.objDetDpmHandle,
                                 DPC_RANGING_IOCTL__STATIC_PRE_START_CFG,
                                 &objDetPreStartDspCfg,
                                 sizeof (DPC_Ranging_PreStartCfg));
            DebugP_log0("App: DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG is processed \n");

            if (errCode < 0)
            {
                System_printf ("Error: Unable to send DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG [Error:%d]\n", errCode);
                goto exit;
            }
        }
    }
exit:
    return errCode;
}

/**
 *  @b Description
 *  @n
 *      This function is used to start data path to handle chirps from front end.
 *
 *  @retval
 *      Not Applicable.
 */
static void Ranging_dataPathStart (void)
{
    int32_t retVal;

    DebugP_log0("App: Issuing DPM_start\n");

    /* Configure HW LVDS stream for the first sub-frame that will start upon
     * start of frame */
    if (gMmwMssMCB.subFrameCfg[0].lvdsStreamCfg.dataFmt != MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED)
    {
        Ranging_configLVDSHwData(0);
    }

    /* Start the DPM Profile: */
    if ((retVal = DPM_start(gMmwMssMCB.objDetDpmHandle)) < 0)
    {
        /* Error: Unable to start the profile */
        System_printf("Error: Unable to start the DPM [Error: %d]\n", retVal);
        Ranging_debugAssert(0);
    }

    /* Wait until start completed */
    Semaphore_pend(gMmwMssMCB.DPMstartSemHandle, BIOS_WAIT_FOREVER);

    DebugP_log0("App: DPM_start Done (post Semaphore_pend on reportFxn reporting start)\n");
}

/**
 *  @b Description
 *  @n
 *      This function is used to stop data path.
 *
 *  @retval
 *      Not Applicable.
 */
static void Ranging_dataPathStop (void)
{
    int32_t retVal;

    DebugP_log0("App: Issuing DPM_stop\n");

    retVal = DPM_stop (gMmwMssMCB.objDetDpmHandle);
    if (retVal < 0)
    {
        System_printf ("DPM_stop failed[Error code %d]\n", retVal);
        Ranging_debugAssert(0);
    }
}

/**
 *  @b Description
 *  @n
 *      Registered event function to mmwave which is invoked when an event from the
 *      BSS is received.
 *
 *  @param[in]  msgId
 *      Message Identifier
 *  @param[in]  sbId
 *      Subblock identifier
 *  @param[in]  sbLen
 *      Length of the subblock
 *  @param[in]  payload
 *      Pointer to the payload buffer
 *
 *  @retval
 *      Always return 0
 */
static int32_t Ranging_eventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload)
{
    uint16_t asyncSB = RL_GET_SBID_FROM_UNIQ_SBID(sbId);

    /* Process the received message: */
    switch (msgId)
    {
        case RL_RF_ASYNC_EVENT_MSG:
        {
            /* Received Asychronous Message: */
            switch (asyncSB)
            {
                case RL_RF_AE_CPUFAULT_SB:
                {
                    rlCpuFault_t *rfCpuFault = (rlCpuFault_t *)payload;
                    System_printf("Debug: CPU Fault has been detected\n");
                    CLI_write("ERROR: Fault \n type: %d, lineNum: %d, LR: 0x%x \n"
                                "PrevLR: 0x%x, spsr: 0x%x, sp: 0x%x, PC: 0x%x \n"
                                "Status: 0x%x, Source: %d, AxiErrType: %d, AccType: %d, Recovery Type: %d \n",
                                rfCpuFault->faultType,
                                rfCpuFault->lineNum,
                                rfCpuFault->faultLR,
                                rfCpuFault->faultPrevLR,
                                rfCpuFault->faultSpsr,
                                rfCpuFault->faultSp,
                                rfCpuFault->faultAddr,
                                rfCpuFault->faultErrStatus,
                                rfCpuFault->faultErrSrc,
                                rfCpuFault->faultAxiErrType,
                                rfCpuFault->faultAccType,
                                rfCpuFault->faultRecovType);
                    Ranging_debugAssert(0);
                    break;
                }
                case RL_RF_AE_ESMFAULT_SB:
                {
                    CLI_write("ERROR: ESM Fault. Group1:[0x%x] Group2:[0x%x]\n",
                                ((rlBssEsmFault_t *)payload)->esmGrp1Err,
                                ((rlBssEsmFault_t *)payload)->esmGrp2Err);
                    Ranging_debugAssert(0);
                    break;
                }
                case RL_RF_AE_ANALOG_FAULT_SB:
                {
                    Ranging_debugAssert(0);
                    break;
                }
                case RL_RF_AE_INITCALIBSTATUS_SB:
                {
                    rlRfInitComplete_t*  ptrRFInitCompleteMessage;
                    uint32_t            calibrationStatus;

                    /* Get the RF-Init completion message: */
                    ptrRFInitCompleteMessage = (rlRfInitComplete_t*)payload;
                    calibrationStatus = ptrRFInitCompleteMessage->calibStatus & 0x1FFFU;

                    /* Display the calibration status: */
                    CLI_write ("Debug: Init Calibration Status = 0x%x\n", calibrationStatus);
                    break;
                }
                case RL_RF_AE_FRAME_TRIGGER_RDY_SB:
                {
                    gMmwMssMCB.stats.frameTriggerReady++;
                    break;
                }
                case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
                {
                    System_printf("Debug: Monitoring FAIL Report received \n");
                    gMmwMssMCB.stats.failedTimingReports++;
                    break;
                }
                case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB:
                {                    
                    gMmwMssMCB.stats.calibrationReports++;
                    break;
                }
                case RL_RF_AE_FRAME_END_SB:
                {
                    gMmwMssMCB.stats.sensorStopped++;
                    DebugP_log0("App: BSS stop (frame end) received\n");

                    Ranging_dataPathStop();
                    break;
                }
                default:
                {
                    System_printf ("Error: Asynchronous Event SB Id %d not handled\n", asyncSB);
                    break;
                }
            }
            break;
        }
        /* Async Event from MMWL */
        case RL_MMWL_ASYNC_EVENT_MSG:
        {
            switch (asyncSB)
            {
                case RL_MMWL_AE_MISMATCH_REPORT:
                {
                    /* link reports protocol error in the async report from BSS */
                    Ranging_debugAssert(0);
                    break;
                }            
                case RL_MMWL_AE_INTERNALERR_REPORT:
                {
                    /* link reports internal error during BSS communication */
                    Ranging_debugAssert(0);
                    break;
                }
            }
            break;
        }
        default:
        {
            System_printf ("Error: Asynchronous message %d is NOT handled\n", msgId);
            break;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      DPM Registered Report Handler. The DPM Module uses this registered function to notify
 *      the application about DPM reports.
 *
 *  @param[in]  reportType
 *      Report Type
 *  @param[in]  instanceId
 *      Instance Identifier which generated the report
 *  @param[in]  errCode
 *      Error code if any.
 *  @param[in] arg0
 *      Argument 0 interpreted with the report type
 *  @param[in] arg1
 *      Argument 1 interpreted with the report type
 *
 *  @retval
 *      Not Applicable.
 */
static void Ranging_DPC_reportFxn
(
    DPM_Report  reportType,
    uint32_t    instanceId,
    int32_t     errCode,
    uint32_t    arg0,
    uint32_t    arg1
)
{
    /* Only errors are logged on the console: */
    if ((errCode != 0) )
    {
        /* Error: Detected log on the console and die all errors are FATAL currently. */
        System_printf ("Error: DPM Report %d received with error:%d arg0:0x%x arg1:0x%x\n",
                        reportType, errCode, arg0, arg1);
        DebugP_assert (0);
    }

    /* Processing further is based on the reports received: This is the control of the profile
     * state machine: */
    switch (reportType)
    {
        case DPM_Report_IOCTL:
        {
            /*****************************************************************
             * DPC has been configured without an error:
             * - This is an indication that the profile configuration commands
             *   went through without any issues.
             *****************************************************************/
            DebugP_log1("App: DPM Report IOCTL, command = %d\n", arg0);

            if (arg0 == DPC_RANGING_IOCTL__STATIC_PRE_START_CFG)
            {
                DPC_Ranging_PreStartCfg *cfg;
                DPC_Ranging_DPC_IOCTL_preStartCfg_memUsage *memUsage;

                cfg = (DPC_Ranging_PreStartCfg*)arg1;

                memUsage = &cfg->memUsage;

                System_printf("============ Heap Memory Stats ============\n");
                System_printf("%20s %12s %12s %12s %12s\n", " ", "Size", "Used", "Free", "DPCUsed");
                System_printf("%20s %12d %12d %12d %12d\n", "System Heap(L2)",
                              memUsage->SystemHeapTotal, memUsage->SystemHeapUsed,
                              memUsage->SystemHeapTotal - memUsage->SystemHeapUsed,
                              memUsage->SystemHeapDPCUsed);

                System_printf("%20s %12d %12d %12d\n", "L3",
                              memUsage->L3RamTotal,
                              memUsage->L3RamUsage,
                              memUsage->L3RamTotal - memUsage->L3RamUsage);

                System_printf("%20s %12d %12d %12d\n", "localRam(L2)",
                              memUsage->CoreL2RamTotal,
                              memUsage->CoreL2RamUsage,
                              memUsage->CoreL2RamTotal - memUsage->CoreL2RamUsage);

                System_printf("%20s %12d %12d %12d\n", "localRam(L1)",
                              memUsage->CoreL1RamTotal,
                              memUsage->CoreL1RamUsage,
                              memUsage->CoreL1RamTotal - memUsage->CoreL1RamUsage);
            }

            switch(arg0)
            {
                /* The following ioctls take longer time to finish. It causes DPM to queue IOCTL requests on DSS before
                 * they are handled. However DPM has limited pipe queues, hence adding sync points in demo to avoid 
                 * sending too many such ioctls to DSS at a time.
                 * The semaphore blocks CLI task to wait for the response from DSS before sending the next ioctl.
                 */
                case DPC_RANGING_IOCTL__STATIC_PRE_START_COMMON_CFG:
                case DPC_RANGING_IOCTL__STATIC_PRE_START_CFG:
                    Semaphore_post(gMmwMssMCB.DPMioctlSemHandle);
                    break;
                default:
                    break;
            }
            break;
        }
        case DPM_Report_DPC_STARTED:
        {
            /*****************************************************************
             * DPC has been started without an error:
             * - notify sensor management task that DPC is started.
             *****************************************************************/
            DebugP_log0("App: DPM Report DPC Started\n");
            gMmwMssMCB.stats.dpmStartEvents++;
            Semaphore_post(gMmwMssMCB.DPMstartSemHandle);
            break;
        }
        case DPM_Report_NOTIFY_DPC_RESULT:
        {
            /*****************************************************************
             * datapath has finished frame processing, results are reported
             *****************************************************************/
            DPM_Buffer*     ptrResult;

            /* Get the result: */
            ptrResult = (DPM_Buffer*)arg0;

            Ranging_handleObjectDetResult(ptrResult);
            break;
        }
        case DPM_Report_DPC_ASSERT:
        {
            DPM_DPCAssert*  ptrAssert;

            /*****************************************************************
             * DPC Fault has been detected:
             * - This implies that the DPC has crashed.
             * - The argument0 points to the DPC assertion information
             *****************************************************************/
            ptrAssert = (DPM_DPCAssert*)arg0;
            CLI_write("Obj Det DPC Exception: %s, line %d.\n", ptrAssert->fileName,
                       ptrAssert->lineNum);
            break;
        }
        case DPM_Report_DPC_STOPPED:
        {
            /*****************************************************************
             * DPC has been stopped without an error:
             * - This implies that the DPC can either be reconfigured or
             *   restarted.
             *****************************************************************/
            DebugP_log0("App: DPM Report DPC Stopped\n");
            gMmwMssMCB.stats.dpmStopEvents++;
            /* every sensor stop should cause 2 DPM stop events due to distributed domain 
               Wait for both the events before proceeding with remaining steps */
            Semaphore_post(gMmwMssMCB.DPMstopSemHandle);
            break;
        }
        case DPM_Report_DPC_INFO:
        case DPM_Report_NOTIFY_DPC_RESULT_ACKED:
        {
            /* Currently objDetDsp does not use this feature. */
            break;
        }
        default:
        {
            DebugP_assert (0);
            break;
        }
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      Utility function to get next sub-frame index
 *
 *  @param[in] currentIndx      Current sub-frame index
 *  @param[in] numSubFrames     Number of sub-frames
 *
 *  @retval
 *      Index of next sub-frame.
 */
static uint8_t Ranging_getNextSubFrameIndx(uint8_t currentIndx, uint8_t numSubFrames)
{
    uint8_t nextIndx;

    if (currentIndx == (numSubFrames - 1))
    {
        nextIndx = 0;
    }
    else
    {
        nextIndx = currentIndx + 1;
    }
    return(nextIndx);
}

/**
 *  @b Description
 *  @n
 *      Utility function to get previous sub-frame index
 *
 *  @param[in] currentIndx      Current sub-frame index
 *  @param[in] numSubFrames     Number of sub-frames
 *
 *  @retval
 *      Index of previous sub-frame
 */
static uint8_t Ranging_getPrevSubFrameIndx(uint8_t currentIndx, uint8_t numSubFrames)
{
    uint8_t prevIndx;

    if (currentIndx == 0)
    {
        prevIndx = numSubFrames - 1;
    }
    else
    {
        prevIndx = currentIndx - 1;
    }
    return(prevIndx);
}

/**
 *  @b Description
 *  @n
 *      Transmit user data over LVDS interface.
 *
 *  @param[in]  subFrameIndx Sub-frame index
 *  @param[in]  dpcResults   pointer to DPC result
 *
 */
void Ranging_transferLVDSUserData(uint8_t subFrameIndx,
                                  DPC_Ranging_ExecuteResult *dpcResults)
{
    int32_t errCode;
    DPC_Ranging_Stats *stats;
    //DPIF_PointCloudCartesian *objOut;
    //DPIF_PointCloudSideInfo *objOutSideInfo;

    stats = (DPC_Ranging_Stats *) SOC_translateAddress((uint32_t)dpcResults->stats,
                                                 SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
                                                 &errCode);
    DebugP_assert ((uint32_t)stats != SOC_TRANSLATEADDR_INVALID);

//    objOut = (DPIF_PointCloudCartesian *) SOC_translateAddress((uint32_t)dpcResults->objOut,
//                                                 SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
//                                                 &errCode);
//    DebugP_assert ((uint32_t)objOut != SOC_TRANSLATEADDR_INVALID);

//    objOutSideInfo = (DPIF_PointCloudSideInfo *) SOC_translateAddress((uint32_t)dpcResults->objOutSideInfo,
//                                                 SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
//                                                 &errCode);
//    DebugP_assert ((uint32_t)objOutSideInfo != SOC_TRANSLATEADDR_INVALID);

    /* Delete previous SW session if it exists. SW session is being
       reconfigured every frame because number of detected objects
       may change from frame to frame which implies that the size of
       the streamed data may change. */
    if(gMmwMssMCB.lvdsStream.swSessionHandle != NULL)
    {
        Ranging_LVDSStreamDeleteSwSession();
    }

    /* Configure SW session for this subframe */
//    if (Ranging_LVDSStreamSwConfig(dpcResults->numObjOut/*,
//                                   objOut,
//                                   objOutSideInfo*/) < 0)
//    {
//        System_printf("Failed LVDS stream SW configuration for sub-frame %d\n", subFrameIndx);
//        Ranging_debugAssert(0);
//        return;
//    }

    /* Populate user data header that will be streamed out*/
    gMmwMssMCB.lvdsStream.userDataHeader.frameNum  = stats->frameStartIntCounter;
//    gMmwMssMCB.lvdsStream.userDataHeader.detObjNum = dpcResults->numObjOut;
    gMmwMssMCB.lvdsStream.userDataHeader.subFrameNum  = (uint16_t) dpcResults->subFrameIdx;

    /* If SW LVDS stream is enabled, start the session here. User data will immediately
       start to stream over LVDS.*/
    if(CBUFF_activateSession (gMmwMssMCB.lvdsStream.swSessionHandle, &errCode) < 0)
    {
        System_printf("Failed to activate CBUFF session for LVDS stream SW. errCode=%d\n",errCode);
        Ranging_debugAssert(0);
        return;
    }
}


uint32_t gts[100];
uint32_t gtsIdx = 0;

/**
 *  @b Description
 *  @n
 *      Function to handle frame processing results from DPC
 *
 *  @param[in] ptrResult      Pointer to DPC result
 *
 *  @retval
 *      Not Applicable.
 */
static void Ranging_handleObjectDetResult
(
    DPM_Buffer  *ptrResult
)
{
    int32_t     retVal;
    DPC_Ranging_ExecuteResultExportedInfo exportInfo;
    DPC_Ranging_ExecuteResult        *dpcResults;
    Ranging_output_message_stats            *frameStats;
    volatile uint32_t                        startTime;
    uint8_t                                  nextSubFrameIdx;
    uint8_t                                  numSubFrames;
    uint8_t                                  currSubFrameIdx;
    uint8_t                                  prevSubFrameIdx;
    Ranging_SubFrameStats                    *currSubFrameStats;
    Ranging_SubFrameStats                    *prevSubFrameStats;

    /*****************************************************************
     * datapath has finished frame processing, results are reported
     *****************************************************************/

    /* Validate DPC results buffer */
    DebugP_assert (ptrResult->size[0] == sizeof(DPC_Ranging_ExecuteResult));

    /* Translate the address: */
    dpcResults = (DPC_Ranging_ExecuteResult *)SOC_translateAddress((uint32_t)ptrResult->ptrBuffer[0],
                                             SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
                                             &retVal);
    DebugP_assert ((uint32_t)dpcResults != SOC_TRANSLATEADDR_INVALID);

    /* Validate timing Info buffer */
    DebugP_assert (ptrResult->size[1] == sizeof(Ranging_output_message_stats));

    numSubFrames = gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames;
    currSubFrameIdx = dpcResults->subFrameIdx;
    prevSubFrameIdx = Ranging_getPrevSubFrameIndx(currSubFrameIdx, numSubFrames);
    currSubFrameStats = &gMmwMssMCB.subFrameStats[currSubFrameIdx];
    prevSubFrameStats = &gMmwMssMCB.subFrameStats[prevSubFrameIdx];

    /*****************************************************************
     * Transmit results
     *****************************************************************/
    startTime = Cycleprofiler_getTimeStamp();
    gts[gtsIdx++]=Cycleprofiler_getTimeStamp();


    /* Translate the address: */
    frameStats = (Ranging_output_message_stats *)SOC_translateAddress((uint32_t)ptrResult->ptrBuffer[1],
                                             SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
                                             &retVal);
    DebugP_assert ((uint32_t)frameStats != SOC_TRANSLATEADDR_INVALID);

    /* Update current frame stats */
    currSubFrameStats->outputStats.interFrameCPULoad = frameStats->interFrameCPULoad;
    currSubFrameStats->outputStats.activeFrameCPULoad= gMmwMssMCB.subFrameStats[currSubFrameIdx].outputStats.activeFrameCPULoad;
    currSubFrameStats->outputStats.interChirpProcessingMargin = frameStats->interChirpProcessingMargin;
    currSubFrameStats->outputStats.interFrameProcessingTime = frameStats->interFrameProcessingTime;
    prevSubFrameStats->outputStats.interFrameProcessingMargin = frameStats->interFrameProcessingMargin;
    currSubFrameStats->outputStats.interFrameProcessingMargin = currSubFrameStats->outputStats.interFrameProcessingMargin -
                                                         (currSubFrameStats->pendingConfigProcTime + currSubFrameStats->subFramePreparationTime);

    if (gMmwMssMCB.subFrameCfg[currSubFrameIdx].lvdsStreamCfg.dataFmt !=
             MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED)
    {
        /* Pend for completion of h/w session, generally this will not wait
         * because of time spent doing inter-frame processing is expected to
         * be bigger than the transmission of the h/w session */
        Semaphore_pend(gMmwMssMCB.lvdsStream.hwFrameDoneSemHandle, BIOS_WAIT_FOREVER);
    }
    gts[gtsIdx++]=Cycleprofiler_getTimeStamp();

    /* Transfer data on LVDS if s/w session is enabled for the current sub-frame */
    if(gMmwMssMCB.subFrameCfg[currSubFrameIdx].lvdsStreamCfg.isSwEnabled == 1)
    {
        Ranging_transferLVDSUserData(currSubFrameIdx, dpcResults);
    }
    gts[gtsIdx++]=Cycleprofiler_getTimeStamp();

    /* Transmit processing results for the frame */
    Ranging_transmitProcessedOutput(gMmwMssMCB.loggingUartHandle,
                                    dpcResults,
                                    &currSubFrameStats->outputStats);
    gts[gtsIdx++]=Cycleprofiler_getTimeStamp();

    /* Wait until s/w session is complete. We expect the LVDS transmission of
     * s/w session to be completed by now because the UART transmission above is slower.
     * Doing the wait immediately after starting the transmission above Ranging_transferLVDSUserData
     * will serialize the LVDS and UART transfers so it is better to do after UART
     * transmission (which is blocking call i.e UART transmission is completed when we exit
     * out of above Ranging_transmitProcessedOutput). Note we cannot replace below code
     * with check for previous sub-frame s/w session completion before the
     * Ranging_transferLVDSUserData above because we need to ensure that
     * current sub-frame/frame's contents are not being read during the
     * next sub-frame/frame transmission, presently the data that is being
     * transmitted is not double buffered to allow this */
    if(gMmwMssMCB.subFrameCfg[currSubFrameIdx].lvdsStreamCfg.isSwEnabled == 1)
    {
        /* Pend completion of s/w session, no wait is expected here */
        Semaphore_pend(gMmwMssMCB.lvdsStream.swFrameDoneSemHandle, BIOS_WAIT_FOREVER);
    }
    gts[gtsIdx++]=Cycleprofiler_getTimeStamp();

    /* Update current frame transmit time */
    currSubFrameStats->outputStats.transmitOutputTime = (Cycleprofiler_getTimeStamp() - startTime)/R4F_CLOCK_MHZ; /* In micro seconds */


    /*****************************************************************
     * Handle dynamic pending configuration
     * For non-advanced frame case:
     *   process all pending dynamic config commands.
     * For advanced-frame case:
     *  Process next sub-frame related pending dynamic config commands.
     *  If the next sub-frame was the first sub-frame of the frame,
     *  then process common (sub-frame independent) pending dynamic config
     *  commands.
     *****************************************************************/
    startTime = Cycleprofiler_getTimeStamp();
    
    nextSubFrameIdx = Ranging_getNextSubFrameIndx(currSubFrameIdx,   numSubFrames);
    if (retVal != 0)
    {
        System_printf ("Error: Executing Pending Dynamic Configuration Commands [Error code %d]\n",
                       retVal);
        Ranging_debugAssert (0);
    }
    currSubFrameStats->pendingConfigProcTime = (Cycleprofiler_getTimeStamp() - startTime)/R4F_CLOCK_MHZ;
    gts[gtsIdx++]=Cycleprofiler_getTimeStamp();
    /*****************************************************************
     * Prepare for subFrame switch
     *****************************************************************/
    if(numSubFrames > 1)
    {
        Ranging_SubFrameCfg  *nextSubFrameCfg;
        uint16_t dummyRxChanOffset[SYS_COMMON_NUM_RX_CHANNEL];

        startTime = Cycleprofiler_getTimeStamp();

        nextSubFrameCfg = &gMmwMssMCB.subFrameCfg[nextSubFrameIdx];

        /* Configure ADC for next sub-frame */
        retVal = Ranging_ADCBufConfig(gMmwMssMCB.adcBufHandle,
                                 gMmwMssMCB.cfg.openCfg.chCfg.rxChannelEn,
                                 nextSubFrameCfg->numChirpsPerChirpEvent,
                                 nextSubFrameCfg->adcBufChanDataSize,
                                 &nextSubFrameCfg->adcBufCfg,
                                 &dummyRxChanOffset[0]);
        if(retVal < 0)
        {
            System_printf("Error: ADCBuf config failed with error[%d]\n", retVal);
            Ranging_debugAssert (0);
        }

        /* Configure HW LVDS stream for this subframe? */
        if(nextSubFrameCfg->lvdsStreamCfg.dataFmt != MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED)
        {
            Ranging_configLVDSHwData(nextSubFrameIdx);
        }

        currSubFrameStats->subFramePreparationTime = (Cycleprofiler_getTimeStamp() - startTime)/R4F_CLOCK_MHZ;
        gts[gtsIdx++]=Cycleprofiler_getTimeStamp();
    }
    else
    {
        currSubFrameStats->subFramePreparationTime = 0;
    }

    /*****************************************************************
     * Send notification to data path after results are handled
     *****************************************************************/
    /* Indicate result consumed and end of frame/sub-frame processing */
    exportInfo.subFrameIdx = currSubFrameIdx;
    retVal = DPM_ioctl (gMmwMssMCB.objDetDpmHandle,
                         DPC_RANGING_IOCTL__DYNAMIC_EXECUTE_RESULT_EXPORTED,
                         &exportInfo,
                         sizeof (DPC_Ranging_ExecuteResultExportedInfo));
    if (retVal < 0) {
        System_printf ("Error: DPM DPC_OBJDET_IOCTL__DYNAMIC_EXECUTE_RESULT_EXPORTED failed [Error code %d]\n",
                       retVal);
        Ranging_debugAssert (0);
    }
    gts[gtsIdx++]=Cycleprofiler_getTimeStamp();

    if (gtsIdx>90) gtsIdx=0;
}

/**
 *  @b Description
 *  @n
 *      DPM Execution Task which executes the DPM Instance which manages the
 *      HL Profiles executing on the MSS.
 *
 *  @retval
 *      Not Applicable.
 */
static void mmwDemo_mssDPMTask(UArg arg0, UArg arg1)
{
    int32_t     errCode;
    DPM_Buffer  result;

    while (1)
    {
        /* Execute the DPM module: */
        errCode = DPM_execute (gMmwMssMCB.objDetDpmHandle, &result);
        if (errCode < 0)
        {
            System_printf ("Error: DPM execution failed [Error code %d]\n", errCode);
        }
    }
}

/**
 *  @b Description
 *  @n
 *      Function to Setup the HSI Clock. Required for LVDS streaming.
 *
 *  @retval
 *      0  - Success.
 *      <0 - Failed with errors
 */
int32_t Ranging_mssSetHsiClk(void)
{
    rlDevHsiClk_t                           hsiClkgs;
    int32_t                                 retVal;

    /*************************************************************************************
     * Setup the HSI Clock through the mmWave Link:
     *************************************************************************************/
    memset ((void*)&hsiClkgs, 0, sizeof(rlDevHsiClk_t));

    /* Setup the HSI Clock as per the Radar Interface Document:
     * - This is set to 600Mhz DDR Mode */
    hsiClkgs.hsiClk = 0x9;

    /* Setup the HSI in the radar link: */
    retVal = rlDeviceSetHsiClk(RL_DEVICE_MAP_CASCADED_1, &hsiClkgs);
    if (retVal != RL_RET_CODE_OK)
    {
        /* Error: Unable to set the HSI clock */
        System_printf ("Error: Setting up the HSI Clock Failed [Error %d]\n", retVal);
        return -1;
    }

    /*The delay below is needed only if the DCA1000EVM is being used to capture the data traces.
      This is needed because the DCA1000EVM FPGA needs the delay to lock to the
      bit clock before they can start capturing the data correctly. */
    Task_sleep(HSI_DCA_MIN_DELAY_MSEC);

    return 0;
}

/**************************************************************************
 ******************** Millimeter Wave Demo sensor management Functions **********
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      mmw demo helper Function to do one time sensor initialization. 
 *      User need to fill gMmwMssMCB.cfg.openCfg before calling this function
 *
 *  @param[in]  isFirstTimeOpen     If true then issues MMwave_open
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t Ranging_openSensor(bool isFirstTimeOpen)
{
    int32_t             errCode;
    MMWave_ErrorLevel   errorLevel;
    int16_t             mmWaveErrorCode;
    int16_t             subsysErrorCode;
    int32_t             retVal;
    MMWave_CalibrationData     calibrationDataCfg;
    MMWave_CalibrationData     *ptrCalibrationDataCfg;

    /*  Open mmWave module, this is only done once */
    if (isFirstTimeOpen == true)
    {
        
        System_printf ("Debug: Sending rlRfSetLdoBypassConfig with %d %d %d\n",
                                            gRFLdoBypassCfg.ldoBypassEnable,
                                            gRFLdoBypassCfg.supplyMonIrDrop,
                                            gRFLdoBypassCfg.ioSupplyIndicator);
        retVal = rlRfSetLdoBypassConfig(RL_DEVICE_MAP_INTERNAL_BSS, (rlRfLdoBypassCfg_t*)&gRFLdoBypassCfg);        
        if(retVal != 0)
        {            
            System_printf("Error: rlRfSetLdoBypassConfig retVal=%d\n", retVal);
            return -1;
        }

        /*  Open mmWave module, this is only done once */
        /* Setup the calibration frequency */
        gMmwMssMCB.cfg.openCfg.freqLimitLow = 600U;
        gMmwMssMCB.cfg.openCfg.freqLimitHigh = 640U;

        /* start/stop async events */
        gMmwMssMCB.cfg.openCfg.disableFrameStartAsyncEvent = false;
        gMmwMssMCB.cfg.openCfg.disableFrameStopAsyncEvent  = false;

        /* No custom calibration: */
        gMmwMssMCB.cfg.openCfg.useCustomCalibration        = false;
        gMmwMssMCB.cfg.openCfg.customCalibrationEnableMask = 0x0;

        /* calibration monitoring base time unit
         * setting it to one frame duration as the demo doesnt support any 
         * monitoring related functionality
         */
        gMmwMssMCB.cfg.openCfg.calibMonTimeUnit            = 1;

        if( (gMmwMssMCB.calibCfg.saveEnable != 0) &&
		(gMmwMssMCB.calibCfg.restoreEnable != 0) )
        {
            /* Error: only one can be enabled at at time */
            System_printf ("Error: Ranging failed with both save and restore enabled.\n");
            return -1;
	    }

        if(gMmwMssMCB.calibCfg.restoreEnable != 0)
        {
            if(Ranging_calibRestore(&gCalibDataStorage) < 0)
            {
	            System_printf ("Error: Ranging failed restoring calibration data from flash.\n");
	            return -1;
            }

            /*  Boot calibration during restore: Disable calibration for:
                 - Rx gain,
                 - Rx IQMM,
                 - Tx phase shifer,
                 - Tx Power

                 The above calibration data will be restored from flash. Since they are calibrated in a control
                 way to avoid interference and spec violations.
                 In this demo, other bit fields(except the above) are enabled as indicated in customCalibrationEnableMask to perform boot time
                 calibration. The boot time calibration will overwrite the restored calibration data from flash.
                 However other bit fields can be disabled and calibration data can be restored from flash as well.

                 Note: In this demo, calibration masks are enabled for all bit fields when "saving" the data.
            */
            gMmwMssMCB.cfg.openCfg.useCustomCalibration        = true;
            gMmwMssMCB.cfg.openCfg.customCalibrationEnableMask = 0x1F0U;

            calibrationDataCfg.ptrCalibData = &gCalibDataStorage.calibData;
            calibrationDataCfg.ptrPhaseShiftCalibData = &gCalibDataStorage.phaseShiftCalibData;
            ptrCalibrationDataCfg = &calibrationDataCfg;
        }
        else
        {
            ptrCalibrationDataCfg = NULL;
        }

        /* Open the mmWave module: */
        if (MMWave_open (gMmwMssMCB.ctrlHandle, &gMmwMssMCB.cfg.openCfg, ptrCalibrationDataCfg, &errCode) < 0)
        {
            /* Error: decode and Report the error */
            MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
            System_printf ("Error: mmWave Open failed [Error code: %d Subsystem: %d]\n",
                            mmWaveErrorCode, subsysErrorCode);
            return -1;
        }

        /* Save calibration data in flash */
        if(gMmwMssMCB.calibCfg.saveEnable != 0)
        {

            retVal = rlRfCalibDataStore(RL_DEVICE_MAP_INTERNAL_BSS, &gCalibDataStorage.calibData);
            if(retVal != RL_RET_CODE_OK)
            {
                /* Error: Calibration data restore failed */
	         System_printf("MSS demo failed rlRfCalibDataStore with Error[%d]\n", retVal);
                return -1;
            }

#if (defined(SOC_XWR18XX) || defined(SOC_XWR68XX))

        /* update txIndex in all chunks to get data from all Tx.
           This should be done regardless of num TX channels enabled in MMWave_OpenCfg_t::chCfg or number of Tx
           application is interested in. Data for all existing Tx channels should be retrieved
           from RadarSS and in the order as shown below.
           RadarSS will return non-zero phase shift values for all the channels enabled via
           MMWave_OpenCfg_t::chCfg and zero phase shift values for channels disabled in MMWave_OpenCfg_t::chCfg */
            gCalibDataStorage.phaseShiftCalibData.PhShiftcalibChunk[0].txIndex = 0;
            gCalibDataStorage.phaseShiftCalibData.PhShiftcalibChunk[1].txIndex = 1;
            gCalibDataStorage.phaseShiftCalibData.PhShiftcalibChunk[2].txIndex = 2;

            /* Basic validation passed: Restore the phase shift calibration data */
            retVal = rlRfPhShiftCalibDataStore(RL_DEVICE_MAP_INTERNAL_BSS, &(gCalibDataStorage.phaseShiftCalibData));
            if (retVal != RL_RET_CODE_OK)
            {
                /* Error: Phase shift Calibration data restore failed */
	         System_printf("MSS demo failed rlRfPhShiftCalibDataStore with Error[%d]\n", retVal);
                return retVal;
            }
#endif
            /* Save data in flash */
            retVal = Ranging_calibSave(&gMmwMssMCB.calibCfg.calibDataHdr, &gCalibDataStorage);
            if(retVal < 0)
            {
                return retVal;
            }
        }

        /*Set up HSI clock*/
        if(Ranging_mssSetHsiClk() < 0)
        {
            System_printf ("Error: Ranging_mssSetHsiClk failed.\n");
            return -1;
        }

        /* Open the datapath modules that runs on MSS */
        Ranging_dataPathOpen();
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      MMW demo helper Function to configure sensor. User need to fill gMmwMssMCB.cfg.ctrlCfg and
 *      add profiles/chirp to mmWave before calling this function
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t Ranging_configSensor(void)
{
    int32_t     errCode = 0;

    /* Prepare BPM configuration */
    if(Ranging_bpmConfig() < 0)
    {
        System_printf ("Error: MMWDemoMSS mmWave BPM Configuration failed\n");
        return -1;
    }

    /* Configure the mmWave module: */
    if (MMWave_config (gMmwMssMCB.ctrlHandle, &gMmwMssMCB.cfg.ctrlCfg, &errCode) < 0)
    {
        MMWave_ErrorLevel   errorLevel;
        int16_t             mmWaveErrorCode;
        int16_t             subsysErrorCode;

        /* Error: Report the error */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
        System_printf ("Error: mmWave Config failed [Error code: %d Subsystem: %d]\n",
                        mmWaveErrorCode, subsysErrorCode);
    }
    else
    {
        errCode = Ranging_dataPathConfig();
    }

    return errCode;
}

/**
 *  @b Description
 *  @n
 *      mmw demo helper Function to start sensor.
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t Ranging_startSensor(void)
{
    int32_t     errCode;
    MMWave_CalibrationCfg   calibrationCfg;

    memset(gts,0,100*sizeof(uint32_t));
    gtsIdx=0;
    /*****************************************************************************
     * Data path :: start data path first - this will pend for DPC to ack
     *****************************************************************************/
    Ranging_dataPathStart();

    /*****************************************************************************
     * RF :: now start the RF and the real time ticking
     *****************************************************************************/
    /* Initialize the calibration configuration: */
    memset ((void *)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));
    /* Populate the calibration configuration: */
    calibrationCfg.dfeDataOutputMode = gMmwMssMCB.cfg.ctrlCfg.dfeDataOutputMode;
    calibrationCfg.u.chirpCalibrationCfg.enableCalibration    = false;
    calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity    = false;
    calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 10U;

    DebugP_log0("App: MMWave_start Issued\n");

    System_printf("Starting Sensor (issuing MMWave_start)\n");

    /* Start the mmWave module: The configuration has been applied successfully. */
    if (MMWave_start(gMmwMssMCB.ctrlHandle, &calibrationCfg, &errCode) < 0)
    {
        MMWave_ErrorLevel   errorLevel;
        int16_t             mmWaveErrorCode;
        int16_t             subsysErrorCode;

        /* Error/Warning: Unable to start the mmWave module */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
        System_printf ("Error: mmWave Start failed [mmWave Error: %d Subsys: %d]\n", mmWaveErrorCode, subsysErrorCode);        
        /* datapath has already been moved to start state; so either we initiate a cleanup of start sequence or
           assert here and re-start from the beginning. For now, choosing the latter path */ 
        Ranging_debugAssert(0);
        return -1;
    }

    /*****************************************************************************
     * The sensor has been started successfully. Switch on the LED 
     *****************************************************************************/
    GPIO_write (gMmwMssMCB.cfg.platformCfg.SensorStatusGPIO, 1U);

    gMmwMssMCB.sensorStartCount++;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Epilog processing after sensor has stopped
 *
 *  @retval None
 */
static void Ranging_sensorStopEpilog(void)
{
    Task_Stat stat;
    Hwi_StackInfo stackInfo;
    Bool stackOverflow;

    /* Print task statistics, note data path has completely stopped due to
     * end of frame, so we can do non-real time processing like prints on
     * console */
    System_printf("Data Path Stopped (last frame processing done)\n");

    System_printf("============================================\n");
    System_printf("MSS Task Stack Usage (Note: Task Stack Usage) ==========\n");
    System_printf("%20s %12s %12s %12s\n", "Task Name", "Size", "Used", "Free");


    Task_stat(gMmwMssMCB.taskHandles.initTask, &stat);
    System_printf("%20s %12d %12d %12d\n", "Init",
                  stat.stackSize, stat.used, stat.stackSize - stat.used);

    Task_stat(gMmwMssMCB.taskHandles.mmwaveCtrl, &stat);
    System_printf("%20s %12d %12d %12d\n", "Mmwave Control",
                  stat.stackSize, stat.used, stat.stackSize - stat.used);

    Task_stat(gMmwMssMCB.taskHandles.objDetDpmTask, &stat);
    System_printf("%20s %12d %12d %12d\n", "ObjDet DPM",
                  stat.stackSize, stat.used, stat.stackSize - stat.used);

    System_printf("HWI Stack (same as System Stack) Usage ============\n");
    stackOverflow = Hwi_getStackInfo(&stackInfo, TRUE);
    if (stackOverflow == TRUE)
    {
        System_printf("HWI Stack overflowed\n");
        Ranging_debugAssert(0);
    }
    else
    {
        System_printf("%20s %12s %12s %12s\n", " ", "Size", "Used", "Free");
        System_printf("%20s %12d %12d %12d\n", " ",
                      stackInfo.hwiStackSize, stackInfo.hwiStackPeak,
                      stackInfo.hwiStackSize - stackInfo.hwiStackPeak);
    }
}


/**
 *  @b Description
 *  @n
 *      Stops the RF and datapath for the sensor. Blocks until both operation are completed.
 *      Prints epilog at the end.
 *
 *  @retval  None
 */
void Ranging_stopSensor(void)
{
    int32_t errCode;

    /* Stop sensor RF , data path will be stopped after RF stop is completed */
    Ranging_mmWaveCtrlStop();

    /* Wait until DPM_stop is completed */
    Semaphore_pend(gMmwMssMCB.DPMstopSemHandle, BIOS_WAIT_FOREVER);

    /* Delete any active streaming session */
    if(gMmwMssMCB.lvdsStream.hwSessionHandle != NULL)
    {
        /* Evaluate need to deactivate h/w session:
         * One sub-frame case:
         *   if h/w only enabled, deactivation never happened, hence need to deactivate
         *   if h/w and s/w both enabled, then s/w would leave h/w activated when it is done
         *   so need to deactivate
         *   (only s/w enabled cannot be the case here because we are checking for non-null h/w session)
         * Multi sub-frame case:
         *   Given stop, we must have re-configured the next sub-frame by now which is next of the
         *   last sub-frame i.e we must have re-configured sub-frame 0. So if sub-frame 0 had
         *   h/w enabled, then it is left in active state and need to deactivate. For all
         *   other cases, h/w was already deactivated when done.
         */
        if ((gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames == 1) ||
            ((gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames > 1) &&
             (gMmwMssMCB.subFrameCfg[0].lvdsStreamCfg.dataFmt != MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED))
           )
        {
            if (CBUFF_deactivateSession(gMmwMssMCB.lvdsStream.hwSessionHandle, &errCode) < 0)
            {
                System_printf("CBUFF_deactivateSession failed with errorCode = %d\n", errCode);
                Ranging_debugAssert(0);
            }
        }
        Ranging_LVDSStreamDeleteHwSession();
    }

    /* Delete s/w session if it exists. S/w session never needs to be deactivated in stop because
     * it always (unconditionally) deactivates itself upon completion.
     */
    if(gMmwMssMCB.lvdsStream.swSessionHandle != NULL)
    {
        Ranging_LVDSStreamDeleteSwSession();
    }

    /* Print epilog */
    Ranging_sensorStopEpilog();

    /* The sensor has been stopped successfully. Switch off the LED */
    GPIO_write (gMmwMssMCB.cfg.platformCfg.SensorStatusGPIO, 0U);

    gMmwMssMCB.sensorStopCount++;

    /* print for user */
    System_printf("Sensor has been stopped: startCount: %d stopCount %d\n",
                  gMmwMssMCB.sensorStartCount,gMmwMssMCB.sensorStopCount);
}

/**************************************************************************
 ******************** Millimeter Wave Demo init Functions ************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Platform specific hardware initialization.
 *
 *  @param[in]  config     Platform initialization configuraiton
 *
 *  @retval
 *      Not Applicable.
 */
static void Ranging_platformInit(Ranging_platformCfg *config)
{

    /* Setup the PINMUX to bring out the UART-1 */
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINN5_PADBE, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);    
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINN5_PADBE, SOC_XWR68XX_PINN5_PADBE_MSS_UARTA_TX);
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINN4_PADBD, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);    
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINN4_PADBD, SOC_XWR68XX_PINN4_PADBD_MSS_UARTA_RX);

    /* Setup the PINMUX to bring out the UART-3 */
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINF14_PADAJ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINF14_PADAJ, SOC_XWR68XX_PINF14_PADAJ_MSS_UARTB_TX);

    /**********************************************************************
     * Setup the PINMUX:
     * - GPIO Output: Configure pin K13 as GPIO_2 output
     **********************************************************************/
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINK13_PADAZ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINK13_PADAZ, SOC_XWR68XX_PINK13_PADAZ_GPIO_2);

    /**********************************************************************
     * Setup the PINMUX:
     * - for QSPI Flash
     **********************************************************************/
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINR12_PADAP, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINR12_PADAP, SOC_XWR68XX_PINR12_PADAP_QSPI_CLK);

    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINP11_PADAQ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINP11_PADAQ, SOC_XWR68XX_PINP11_PADAQ_QSPI_CSN);

    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINR13_PADAL, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINR13_PADAL, SOC_XWR68XX_PINR13_PADAL_QSPI_D0);

    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINN12_PADAM, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINN12_PADAM, SOC_XWR68XX_PINN12_PADAM_QSPI_D1);

    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINR14_PADAN, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINR14_PADAN, SOC_XWR68XX_PINR14_PADAN_QSPI_D2);

    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINP12_PADAO, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINP12_PADAO, SOC_XWR68XX_PINP12_PADAO_QSPI_D3);

    /**********************************************************************
     * Setup the GPIO:
     * - GPIO Output: Configure pin K13 as GPIO_2 output
     **********************************************************************/
    config->SensorStatusGPIO    = SOC_XWR68XX_GPIO_2;

    /* Initialize the DEMO configuration: */
    config->sysClockFrequency   = MSS_SYS_VCLK;
    config->loggingBaudRate     = 921600;
    config->commandBaudRate     = 115200;

    /**********************************************************************
     * Setup the DS3 LED on the EVM connected to GPIO_2
     **********************************************************************/
    GPIO_setConfig (config->SensorStatusGPIO, GPIO_CFG_OUTPUT);
}

/**
 *  @b Description
 *  @n
 *      Calibration save/restore initialization
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Ranging_calibInit(void)
{
    int32_t          retVal = 0;
    rlVersion_t    verArgs;

    /* Initialize verArgs */
    memset((void *)&verArgs, 0, sizeof(rlVersion_t));

    /* Get the version string: */
    retVal = rlDeviceGetVersion(RL_DEVICE_MAP_INTERNAL_BSS, &verArgs);
    if (retVal < 0)
    {
        System_printf ("Error: Unable to get the device version from mmWave link [Error %d]\n", retVal);
        return -1;
    }

    /* Calibration save/restore init */
    gMmwMssMCB.calibCfg.sizeOfCalibDataStorage = sizeof(Ranging_calibData);
    gMmwMssMCB.calibCfg.calibDataHdr.magic = MMWDEMO_CALIB_STORE_MAGIC;
    memcpy((void *)& gMmwMssMCB.calibCfg.calibDataHdr.linkVer, (void *)&verArgs.mmWaveLink, sizeof(rlSwVersionParam_t));
    memcpy((void *)& gMmwMssMCB.calibCfg.calibDataHdr.radarSSVer, (void *)&verArgs.rf, sizeof(rlFwVersionParam_t));

    /* Check if Calibration data is over the Reserved storage */
    if(gMmwMssMCB.calibCfg.sizeOfCalibDataStorage   <= MMWDEMO_CALIB_FLASH_SIZE)
    {
	    gMmwMssMCB.calibCfg.calibDataHdr.hdrLen = sizeof(Ranging_calibDataHeader);
	    gMmwMssMCB.calibCfg.calibDataHdr.dataLen= sizeof(Ranging_calibData) - sizeof(Ranging_calibDataHeader);

           /* Resets calibration data */
           memset((void *)&gCalibDataStorage, 0, sizeof(Ranging_calibData));

	    retVal = mmwDemo_flashInit();
    }
    else
    {
        System_printf ("Error: Calibration data size is bigger than reserved size\n");
        retVal = -1;
    }

    return retVal;

}

/**
 *  @b Description
 *  @n
 *      This function retrieves the calibration data from front end and saves it in flash.
 *
 *  @param[in]  ptrCalibDataHdr     	Pointer to Calibration data header
 *  @param[in]  ptrCalibrationData      Pointer to Calibration data
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Ranging_calibSave(Ranging_calibDataHeader *ptrCalibDataHdr, Ranging_calibData  *ptrCalibrationData)
{
    uint32_t				flashOffset;
    int32_t 				retVal = 0;

    /* Calculate the read size in bytes */
    flashOffset = gMmwMssMCB.calibCfg.flashOffset;

    /* Copy header  */
    memcpy((void *)&(ptrCalibrationData->calibDataHdr), ptrCalibDataHdr, sizeof(Ranging_calibDataHeader));

    /* Flash calibration data */
    retVal = mmwDemo_flashWrite(flashOffset, (uint32_t *)ptrCalibrationData, sizeof(Ranging_calibData));
    if(retVal < 0)
    {
        /* Flash Header failed */
        System_printf ("Error: Ranging failed flashing calibration data with error[%d].\n", retVal);
    }

    return(retVal);
}

/**
 *  @b Description
 *  @n
 *      This function reads calibration data from flash and send it to front end through MMWave_open()
 *
 *  @param[in]  ptrCalibData     	Pointer to Calibration data
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Ranging_calibRestore(Ranging_calibData  *ptrCalibData)
{
    Ranging_calibDataHeader    *pDataHdr;
    int32_t 				retVal = 0;
    uint32_t				flashOffset;

    pDataHdr = &(ptrCalibData->calibDataHdr);

    /* Calculate the read size in bytes */
    flashOffset = gMmwMssMCB.calibCfg.flashOffset;

    /* Read calibration data header */
    if(mmwDemo_flashRead(flashOffset, (uint32_t *)pDataHdr, sizeof(Ranging_calibData) )< 0)
    {
        /* Error: only one can be enable at at time */
        System_printf ("Error: Ranging failed when reading calibration data from flash.\n");
        return -1;
    }

    /* Validate data header */
    if( (pDataHdr->magic != MMWDEMO_CALIB_STORE_MAGIC) ||
         (pDataHdr->hdrLen != gMmwMssMCB.calibCfg.calibDataHdr.hdrLen) ||
         (pDataHdr->dataLen != gMmwMssMCB.calibCfg.calibDataHdr.dataLen) )
    {
        /* Header validation failed */
        System_printf ("Error: Ranging calibration data header validation failed.\n");
        retVal = -1;
    }
    /* Matching mmwLink version:
         In this demo, we would like to save/restore with the matching mmwLink and RF FW version.
         However, this logic can be changed to use data saved from previous mmwLink and FW releases,
         as long as the data format of the calibration data matches.
     */
    else if(memcmp((void *)&pDataHdr->linkVer, (void *)&gMmwMssMCB.calibCfg.calibDataHdr.linkVer, sizeof(rlSwVersionParam_t)) != 0)
    {
        System_printf ("Error: Ranging failed mmwLink version validation when restoring calibration data.\n");
        retVal = -1;
    }
    else if(memcmp((void *)&pDataHdr->radarSSVer, (void *)&gMmwMssMCB.calibCfg.calibDataHdr.radarSSVer, sizeof(rlFwVersionParam_t)) != 0)
    {
        System_printf ("Error: Ranging failed RF FW version validation when restoring calibration data.\n");
        retVal = -1;
    }
    return(retVal);
}
/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
static void Ranging_initTask(UArg arg0, UArg arg1)
{
    int32_t             errCode;
    MMWave_InitCfg      initCfg;
    UART_Params         uartParams;
    Task_Params         taskParams;
    Semaphore_Params    semParams;
    DPM_InitCfg         dpmInitCfg;
    DPC_Ranging_InitParams objDetInitParams;
    int32_t             i;

    /* Debug Message: */
    System_printf("Debug: Launched the Initialization Task\n");

    /*****************************************************************************
     * Initialize the mmWave SDK components:
     *****************************************************************************/
    /* Initialize the UART */
    UART_init();

    /* Initialize the GPIO */
    GPIO_init();

    /* Initialize the Mailbox */
    Mailbox_init(MAILBOX_TYPE_MSS);

    /* Initialize LVDS streaming components */
    if ((errCode = Ranging_LVDSStreamInit()) < 0 )
    {
        System_printf ("Error: MMWDemoDSS LVDS stream init failed with Error[%d]\n",errCode);
        return;
    }

    /* initialize cq configs to invalid profile index to be able to detect
     * unconfigured state of these when monitors for them are enabled.
     */
    for(i = 0; i < RL_MAX_PROFILES_CNT; i++)
    {
        gMmwMssMCB.cqSatMonCfg[i].profileIndx    = (RL_MAX_PROFILES_CNT + 1);
        gMmwMssMCB.cqSigImgMonCfg[i].profileIndx = (RL_MAX_PROFILES_CNT + 1);
    }

    /* Platform specific configuration */
    Ranging_platformInit(&gMmwMssMCB.cfg.platformCfg);

    /*****************************************************************************
     * Open the mmWave SDK components:
     *****************************************************************************/
    /* Setup the default UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.clockFrequency = gMmwMssMCB.cfg.platformCfg.sysClockFrequency;
    uartParams.baudRate       = gMmwMssMCB.cfg.platformCfg.commandBaudRate;
    uartParams.isPinMuxDone   = 1;

    /* Open the UART Instance */
    gMmwMssMCB.commandUartHandle = UART_open(0, &uartParams);
    if (gMmwMssMCB.commandUartHandle == NULL)
    {
        Ranging_debugAssert (0);
        return;
    }

    /* Setup the default UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.clockFrequency = gMmwMssMCB.cfg.platformCfg.sysClockFrequency;
    uartParams.baudRate       = gMmwMssMCB.cfg.platformCfg.loggingBaudRate;
    uartParams.isPinMuxDone   = 1U;

    /* Open the Logging UART Instance: */
    gMmwMssMCB.loggingUartHandle = UART_open(1, &uartParams);
    if (gMmwMssMCB.loggingUartHandle == NULL)
    {
        System_printf("Error: Unable to open the Logging UART Instance\n");
        Ranging_debugAssert (0);
        return;
    }

    /* Create binary semaphores which is used to signal DPM_start/DPM_stop/DPM_ioctl is done
     * to the sensor management task. The signalling (Semaphore_post) will be done
     * from DPM registered report function (which will execute in the DPM execute task context). */
    Semaphore_Params_init(&semParams);
    semParams.mode              = Semaphore_Mode_BINARY;
    gMmwMssMCB.DPMstartSemHandle   = Semaphore_create(0, &semParams, NULL);
    gMmwMssMCB.DPMstopSemHandle   = Semaphore_create(0, &semParams, NULL);
    gMmwMssMCB.DPMioctlSemHandle   = Semaphore_create(0, &semParams, NULL);

    /* Open EDMA driver */
    Ranging_edmaInit();

    /* Use EDMA instance 0 on MSS */
    Ranging_edmaOpen();

    /*****************************************************************************
     * mmWave: Initialization of the high level module
     *****************************************************************************/

    /* Initialize the mmWave control init configuration */
    memset ((void*)&initCfg, 0 , sizeof(MMWave_InitCfg));

    /* Populate the init configuration: */
    initCfg.domain                  = MMWave_Domain_MSS;
    initCfg.socHandle               = gMmwMssMCB.socHandle;
    initCfg.eventFxn                = Ranging_eventCallbackFxn;
    initCfg.linkCRCCfg.useCRCDriver = 1U;
    initCfg.linkCRCCfg.crcChannel   = CRC_Channel_CH1;
    initCfg.cfgMode                 = MMWave_ConfigurationMode_FULL;
    initCfg.executionMode           = MMWave_ExecutionMode_ISOLATION;

    /* Initialize and setup the mmWave Control module */
    gMmwMssMCB.ctrlHandle = MMWave_init (&initCfg, &errCode);
    if (gMmwMssMCB.ctrlHandle == NULL)
    {
        /* Error: Unable to initialize the mmWave control module */
        System_printf ("Error: mmWave Control Initialization failed [Error code %d]\n", errCode);
        Ranging_debugAssert (0);
        return;
    }
    System_printf ("Debug: mmWave Control Initialization was successful\n");

    /* Synchronization: This will synchronize the execution of the control module
     * between the domains. This is a prerequiste and always needs to be invoked. */
    if (MMWave_sync (gMmwMssMCB.ctrlHandle, &errCode) < 0)
    {
        /* Error: Unable to synchronize the mmWave control module */
        System_printf ("Error: mmWave Control Synchronization failed [Error code %d]\n", errCode);
        Ranging_debugAssert (0);
        return;
    }
    System_printf ("Debug: mmWave Control Synchronization was successful\n");

    /*****************************************************************************
     * Launch the mmWave control execution task
     * - This should have a higher priroity than any other task which uses the
     *   mmWave control API
     *****************************************************************************/
    Task_Params_init(&taskParams);
    taskParams.priority  = MMWDEMO_MMWAVE_CTRL_TASK_PRIORITY;
    taskParams.stackSize = 3*1024;
    gMmwMssMCB.taskHandles.mmwaveCtrl = Task_create(Ranging_mmWaveCtrlTask, &taskParams, NULL);

    /*****************************************************************************
     * Initialization of the DPM Module:
     *****************************************************************************/
    /* Setup the configuration: */
    memset ((void *)&dpmInitCfg, 0, sizeof(DPM_InitCfg));
    dpmInitCfg.socHandle        = gMmwMssMCB.socHandle;
    dpmInitCfg.ptrProcChainCfg  = NULL;
    dpmInitCfg.instanceId       = DPC_OBJDET_R4F_INSTANCEID;
    dpmInitCfg.domain           = DPM_Domain_REMOTE;
    dpmInitCfg.reportFxn        = Ranging_DPC_reportFxn;
    dpmInitCfg.arg              = &objDetInitParams;
    dpmInitCfg.argSize          = sizeof(DPC_Ranging_InitParams);

    /* Initialize the DPM Module: */
    gMmwMssMCB.objDetDpmHandle = DPM_init (&dpmInitCfg, &errCode);
    if (gMmwMssMCB.objDetDpmHandle == NULL)
    {
        System_printf ("Error: Unable to initialize the DPM Module [Error: %d]\n", errCode);
        Ranging_debugAssert (0);
        return;
    }

    /* Synchronization: This will synchronize the execution of the datapath module
     * between the domains. This is a prerequiste and always needs to be invoked. */
    while (1)
    {
        int32_t syncStatus;

        /* Get the synchronization status: */
        syncStatus = DPM_synch (gMmwMssMCB.objDetDpmHandle, &errCode);
        if (syncStatus < 0)
        {
            /* Error: Unable to synchronize the framework */
            System_printf ("Error: DPM Synchronization failed [Error code %d]\n", errCode);
            Ranging_debugAssert (0);
            return;
        }
        if (syncStatus == 1)
        {
            /* Synchronization acheived: */
            break;
        }
        /* Sleep and poll again: */
        Task_sleep(1);
    }

    /* Launch the DPM Task */
    Task_Params_init(&taskParams);
    taskParams.priority  = MMWDEMO_DPC_OBJDET_DPM_TASK_PRIORITY;
    taskParams.stackSize = 4*1024;
    gMmwMssMCB.taskHandles.objDetDpmTask = Task_create(mmwDemo_mssDPMTask, &taskParams, NULL);

    /* Calibration save/restore initialization */
    if(Ranging_calibInit()<0)
    {
        System_printf("Error: Calibration data initialization failed \n");
        Ranging_debugAssert (0);
    }

    /*****************************************************************************
     * Initialize the Profiler
     *****************************************************************************/
    Cycleprofiler_init();

    /*****************************************************************************
     * Initialize the CLI Module:
     *****************************************************************************/
    Ranging_CLIInit(MMWDEMO_CLI_TASK_PRIORITY);

    return;
}

/**
 *  @b Description
 *  @n
 *     Function to sleep the R4F using WFI (Wait For Interrupt) instruction.
 *     When R4F has no work left to do,
 *     the BIOS will be in Idle thread and will call this function. The R4F will
 *     wake-up on any interrupt (e.g chirp interrupt).
 *
 *  @retval
 *      Not Applicable.
 */
void Ranging_sleep(void)
{
    /* issue WFI (Wait For Interrupt) instruction */
    asm(" WFI ");
}

#ifdef SYS_COMMON_XWR68XX_LOW_POWER_MODE_EN
/**
 *  @b Description
 *  @n
 *      Idle power sequence: Performs power down and up cycle by performing following functions
 *
 *      -DSP Powered Off
 *      -MSS VCLK slowed to 40 MHz (to allow for CAN functionality)
 *      -RF Analog Power Down
 *      -APLL Power Down
 *
 *      -Wait 1 millisecond
 *
 *      -APLL Power Up
 *      -RF Analog Power Up
 *      -MSS VCLK sped up to 200 MHz
 *
 *
 *
 *      NOTE: This sequence does not power DSP back on
 *
 *
 *  @retval  None
 */
void idle_power_cycle(IdleModeCfg   idleModeCfg)
{
    int32_t             errCode;
    rlReturnVal_t             retVal;
    rlPowerSaveModeCfg_t        data_rf_down;
    rlPowerSaveModeCfg_t        data_rf_up;
    rlPowerSaveModeCfg_t        data_apll_down;
    rlPowerSaveModeCfg_t        data_apll_up;

    //Enable DSP Shutdown
    int8_t enDSPpowerdown = idleModeCfg.enDSPpowerdown;
    //Enable DSS Clock Gate (No Shutdown)
    int8_t enDSSclkgate = idleModeCfg.enDSSclkgate;
    //Enable switching of MSS VCLK
    int8_t enMSSvclkgate = idleModeCfg.enMSSvclkgate;
    //Enable BSS Clock Gate
    int8_t enBSSclkgate = idleModeCfg.enBSSclkgate;
    //Enable Power switching of RF
    int8_t enRFpowerdown = idleModeCfg.enRFpowerdown;
    //Enable Power switching of APLL
    int8_t enAPLLpowerdown = idleModeCfg.enAPLLpowerdown;
    //Enable Power switching of APLL
    int8_t enAPLLGPADCpowerdown = idleModeCfg.enAPLLGPADCpowerdown;

       //OPTIONAL: Delay between successive
       // in microseconds. (Recommended is 0)
    uint32_t ttime = idleModeCfg.idleModeMicroDelay;

       //OPTIONAL: Delay after power down sequence
       // in microseconds. (Recommended is 1000000)
    uint32_t offtime = idleModeCfg.idleModeMicroDelay;


    if(enMSSvclkgate||enAPLLpowerdown||enAPLLGPADCpowerdown)
    {
        //MSSvclkgate and APLLpowerdown components slow
        //system clock from 200 MHz to 40 MHz, so adjusting delays
        //by a factor of 5
        ttime /= 5;
        offtime /=5;
    }

       gMmwMssMCB.sensorState = Ranging_SensorState_INIT;
       MMWave_close(gMmwMssMCB.ctrlHandle,&errCode);

       // The sensor has been stopped successfully. Switch off the LED
           GPIO_write (gMmwMssMCB.cfg.platformCfg.SensorStatusGPIO, 0U);


     //BEGIN POWER DOWN SEQUENCE


       /* The sensor has been stopped successfully. Switch off the LED */
           GPIO_write (gMmwMssMCB.cfg.platformCfg.SensorStatusGPIO, 0U);

       //DSP Powered Off (There is no power on)
      if(enDSPpowerdown == 1)
      {
        xWR6843_dss_power_down();
        SOC_microDelay(ttime); //delay function (default 0 microsecond)
      }

      //DSS Clock Gate
      if(enDSSclkgate == 1)
      {
          xWR6843_dss_clock_gate();
        SOC_microDelay(ttime); //delay function (default 0 microsecond)
      }

      // MSS VCLK slowed to 40 MHz
      if(enMSSvclkgate == 1)
      {
        xWR6843_mss_vclk_40M();
        SOC_microDelay(ttime); //delay function (default 0 microsecond)
      }

      //RF Analog Power Down
     if(enRFpowerdown == 1)
     {


         data_rf_down.lowPwrStateTransCmd = 1;
         retVal = rlSetPowerSaveModeConfig(RL_DEVICE_INDEX_INTERNAL_DSS_MSS, &data_rf_down);
         if (retVal != 0)
         {
             System_printf("RF Off Failed Err: %d\n", retVal);

         }



        SOC_microDelay(ttime); //delay function (default 0 microsecond)
     }

    //APLL + GPADC Power Down
    if(enAPLLGPADCpowerdown == 1)
    {
        data_apll_down.lowPwrStateTransCmd = 5;
        retVal = rlSetPowerSaveModeConfig(RL_DEVICE_INDEX_INTERNAL_DSS_MSS, &data_apll_down);
       if (retVal != 0)
        {
            System_printf("APLL + GPADC Off Failed Err: %d\n", retVal);

        }

        SOC_microDelay(ttime); //delay function (default 0 microsecond)

    }
    //APLL Power Down
    else if(enAPLLpowerdown == 1)
    {

            data_apll_down.lowPwrStateTransCmd = 3;
            retVal = rlSetPowerSaveModeConfig(RL_DEVICE_INDEX_INTERNAL_DSS_MSS, &data_apll_down);
           if (retVal != 0)
            {
                System_printf("APLL Off Failed Err: %d\n", retVal);

            }

            SOC_microDelay(ttime); //delay function (default 0 microsecond)
    }


    //BSS Clock gate
    //(Performed last because RF/APLL functions require BSS Clock)
    if(enBSSclkgate== 1)
    {
    SOC_haltBSS(gMmwMssMCB.socHandle, &errCode);
    }



    // Optional delay
    SOC_microDelay(offtime);



    //BEGIN POWER UP SEQUENCE
    //BSS Clock Ungate

    if(enBSSclkgate== 1)
    {
    SOC_unhaltBSS(gMmwMssMCB.socHandle, &errCode);
    }

    //APLL + GPADC Power Down
    if(enAPLLGPADCpowerdown == 1)
    {
        data_apll_up.lowPwrStateTransCmd = 6;
        retVal = rlSetPowerSaveModeConfig(RL_DEVICE_INDEX_INTERNAL_DSS_MSS, &data_apll_up);
        if (retVal != 0)
        {
            System_printf("APLL ON Failed Err: %d\n", retVal);
        }

      SOC_microDelay(ttime); //delay function (default 0 microsecond)
    }
    //APLL Power Up
    else if(enAPLLpowerdown == 1)
    {

        data_apll_up.lowPwrStateTransCmd = 4;
        retVal = rlSetPowerSaveModeConfig(RL_DEVICE_INDEX_INTERNAL_DSS_MSS, &data_apll_up);
        if (retVal != 0)
        {
            System_printf("APLL ON Failed Err: %d\n", retVal);
        }

      SOC_microDelay(ttime); //delay function (default 0 microsecond)
    }

    //RF Analog Power Up
    if(enRFpowerdown == 1)
    {

     data_rf_up.lowPwrStateTransCmd = 2;
     retVal = rlSetPowerSaveModeConfig(RL_DEVICE_INDEX_INTERNAL_DSS_MSS, &data_rf_up);
      if (retVal != 0)
      {
          System_printf("RF On Failed Err: %d\n", retVal);
      }

      SOC_microDelay(ttime); //delay function (default 0 microsecond)
    }

    //MSS VCLK sped up to 200 MHz
    if(enMSSvclkgate == 1)
    {
        xWR6843_mss_vclk_200M();
        SOC_microDelay(ttime); //delay function (default 0 microsecond)
    }

    //DSS Clock Ungate
    if(enDSSclkgate == 1)
    {
        xWR6843_dss_clock_ungate();
      SOC_microDelay(ttime); //delay function (default 0 microsecond)
    }



    //END POWER UP SEQUENCE


    //Turn LED back on
        GPIO_write (gMmwMssMCB.cfg.platformCfg.SensorStatusGPIO, 1U);

}



/**
 *  @b Description
 *  @n
 *      Idle power sequence: Performs power down and up cycle by performing following functions
 *
 *      -DSP Powered Off
 *      -MSS VCLK slowed to 40 MHz (to allow for CAN functionality)
 *      -RF Analog Power Down
 *      -APLL Power Down
 *
 *      -Wait 1 millisecond
 *
 *      -APLL Power Up
 *      -RF Analog Power Up
 *      -MSS VCLK sped up to 200 MHz
 *
 *
 *
 *      NOTE: This sequence does not power DSP back on
 *
 *
 *  @retval  None
 */
void idle_power_down(IdleModeCfg   idleModeCfg)
{
    int32_t             errCode;
    rlReturnVal_t             retVal;
    rlPowerSaveModeCfg_t        data_rf_down;
    rlPowerSaveModeCfg_t        data_apll_down;

    //Enable DSP Shutdown
    int8_t enDSPpowerdown = idleModeCfg.enDSPpowerdown;
    //Enable DSS Clock Gate (No Shutdown)
    int8_t enDSSclkgate = idleModeCfg.enDSSclkgate;
    //Enable switching of MSS VCLK
    int8_t enMSSvclkgate = idleModeCfg.enMSSvclkgate;
    //Enable BSS Clock Gate
    int8_t enBSSclkgate = idleModeCfg.enBSSclkgate;
    //Enable Power switching of RF
    int8_t enRFpowerdown = idleModeCfg.enRFpowerdown;
    //Enable Power switching of APLL
    int8_t enAPLLpowerdown = idleModeCfg.enAPLLpowerdown;
    //Enable Power switching of APLL
    int8_t enAPLLGPADCpowerdown = idleModeCfg.enAPLLGPADCpowerdown;

       //OPTIONAL: Delay between successive
       // in microseconds. (Recommended is 0)
    uint32_t ttime = idleModeCfg.idleModeMicroDelay;

       //OPTIONAL: Delay after power down sequence
       // in microseconds. (Recommended is 1000000)
    uint32_t offtime = idleModeCfg.idleModeMicroDelay;


    if(enMSSvclkgate||enAPLLpowerdown||enAPLLGPADCpowerdown)
    {
        //MSSvclkgate and APLLpowerdown components slow
        //system clock from 200 MHz to 40 MHz, so adjusting delays
        //by a factor of 5
        ttime /= 5;
        offtime /=5;
    }

       gMmwMssMCB.sensorState = Ranging_SensorState_INIT;
       MMWave_close(gMmwMssMCB.ctrlHandle,&errCode);

       // The sensor has been stopped successfully. Switch off the LED
           GPIO_write (gMmwMssMCB.cfg.platformCfg.SensorStatusGPIO, 0U);


     //BEGIN POWER DOWN SEQUENCE


       /* The sensor has been stopped successfully. Switch off the LED */
           GPIO_write (gMmwMssMCB.cfg.platformCfg.SensorStatusGPIO, 0U);

       //DSP Powered Off (There is no power on)
      if(enDSPpowerdown == 1)
      {
        xWR6843_dss_power_down();
        SOC_microDelay(ttime); //delay function (default 0 microsecond)
      }

      //DSS Clock Gate
      if(enDSSclkgate == 1)
      {
          xWR6843_dss_clock_gate();
        SOC_microDelay(ttime); //delay function (default 0 microsecond)
      }

      // MSS VCLK slowed to 40 MHz
      if(enMSSvclkgate == 1)
      {
        xWR6843_mss_vclk_40M();
        SOC_microDelay(ttime); //delay function (default 0 microsecond)
      }

      //RF Analog Power Down
     if(enRFpowerdown == 1)
     {


         data_rf_down.lowPwrStateTransCmd = 1;
         retVal = rlSetPowerSaveModeConfig(RL_DEVICE_INDEX_INTERNAL_DSS_MSS, &data_rf_down);
         if (retVal != 0)
         {
             System_printf("RF Off Failed Err: %d\n", retVal);

         }



        SOC_microDelay(ttime); //delay function (default 0 microsecond)
     }

    //APLL + GPADC Power Down
    if(enAPLLGPADCpowerdown == 1)
    {
        data_apll_down.lowPwrStateTransCmd = 5;
        retVal = rlSetPowerSaveModeConfig(RL_DEVICE_INDEX_INTERNAL_DSS_MSS, &data_apll_down);
       if (retVal != 0)
        {
            System_printf("APLL + GPADC Off Failed Err: %d\n", retVal);

        }

        SOC_microDelay(ttime); //delay function (default 0 microsecond)

    }
    //APLL Power Down
    else if(enAPLLpowerdown == 1)
    {

            data_apll_down.lowPwrStateTransCmd = 3;
            retVal = rlSetPowerSaveModeConfig(RL_DEVICE_INDEX_INTERNAL_DSS_MSS, &data_apll_down);
           if (retVal != 0)
            {
                System_printf("APLL Off Failed Err: %d\n", retVal);

            }

            SOC_microDelay(ttime); //delay function (default 0 microsecond)
    }


    //BSS Clock gate
    //(Performed last because RF/APLL functions require BSS Clock)
    if(enBSSclkgate== 1)
    {
    SOC_haltBSS(gMmwMssMCB.socHandle, &errCode);
    }



    // Optional delay
    SOC_microDelay(offtime);


    //END OF POWER DOWN SEQUENCE
    //WILL NOT POWER BACK ON
    //UNLESS HARD RESET OF DEVICE

}
#endif /* SYS_COMMON_XWR68XX_LOW_POWER_MODE_EN */

/**
 *  @b Description
 *  @n
 *      Entry point into the Millimeter Wave Demo
 *
 *  @retval
 *      Not Applicable.
 */
int main (void)
{
    Task_Params     taskParams;
    int32_t         errCode;
    SOC_Handle      socHandle;
    SOC_Cfg         socCfg;

    /* Initialize the ESM: Dont clear errors as TI RTOS does it */
    ESM_init(0U);

    /* Initialize the SOC confiugration: */
    memset ((void *)&socCfg, 0, sizeof(SOC_Cfg));

    /* Populate the SOC configuration: */
    socCfg.clockCfg = SOC_SysClock_INIT;
    socCfg.mpuCfg = SOC_MPUCfg_CONFIG;
    socCfg.dssCfg = SOC_DSSCfg_UNHALT;

    /* Initialize the SOC Module: This is done as soon as the application is started
     * to ensure that the MPU is correctly configured. */
    socHandle = SOC_init (&socCfg, &errCode);
    if (socHandle == NULL)
    {
        System_printf ("Error: SOC Module Initialization failed [Error code %d]\n", errCode);
        Ranging_debugAssert (0);
        return -1;
    }

    /* Wait for BSS powerup */
    if (SOC_waitBSSPowerUp(socHandle, &errCode) < 0)
    {
        /* Debug Message: */
        System_printf ("Debug: SOC_waitBSSPowerUp failed with Error [%d]\n", errCode);
        return 0;
    }

    /* Check if the SOC is a secure device */
    if (SOC_isSecureDevice(socHandle, &errCode))
    {
        /* Disable firewall for JTAG and LOGGER (UART) which is needed by all unit tests */
        SOC_controlSecureFirewall(socHandle, 
                                  (uint32_t)(SOC_SECURE_FIREWALL_JTAG | SOC_SECURE_FIREWALL_LOGGER),
                                  SOC_SECURE_FIREWALL_DISABLE,
                                  &errCode);
    }

    /* Initialize and populate the demo MCB */
    memset ((void*)&gMmwMssMCB, 0, sizeof(Ranging_MSS_MCB));

    gMmwMssMCB.socHandle = socHandle;

    /* Debug Message: */
    System_printf ("**********************************************\n");
    System_printf ("Debug: Launching the MMW Demo on MSS\n");
    System_printf ("**********************************************\n");

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    gMmwMssMCB.taskHandles.initTask = Task_create(Ranging_initTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
    return 0;
}

