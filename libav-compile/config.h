#define LIBAV_CONFIGURATION "--toolchain=msvc"
#define LIBAV_LICENSE "LGPL version"
#define restrict __restrict
#define EXTERN_PREFIX "_"

#define ARCH_X86 1       //CISC 
#define ARCH_X86_32 1    //CISC 
#define ARCH_X86_64 0    //CISC 
#define ARCH_IA64 0      //CISC also called Intel Itanium architecture
#define ARCH_M68K 0      //CISC The Motorola 68000 ("'sixty-eight-thousand'"; also called the m68k or Motorola 68k, "sixty-eight-k")
#define ARCH_ARM 0       //RISC 
#define ARCH_AARCH64 0   //RISC ARM ARCH64
#define ARCH_AVR32 0     //RISC Atmel AVR: a 32-bit RISC microprocessor architecture designed by Atmel
#define ARCH_AVR32_AP 0  //RISC On April 10, 2012 Atmel announced the End of Life of AP7 Core devices on 4/4/2013
#define ARCH_AVR32_UC 0  //RISC UC3 Core
#define ARCH_BFIN 0      //RISC Blackfin处理器基于由ADI和Intel公司联合开发
#define ARCH_MIPS 0      //RISC Microprocessor without interlocked piped stages
#define ARCH_MIPS64 0    //RISC Microprocessor without interlocked piped stages
#define ARCH_PARISC 0    //RISC HP（惠普）公司的RISC芯片PA-RISC于1986年问世,被惠普公司与英特尔联合开发的Itanium架构所取代
#define ARCH_PPC 0       //RISC POWER是1991年，Apple（苹果电脑）、IBM、Motorola（摩托罗拉）组成的AIM联盟所发展出的微处理器架构
#define ARCH_PPC64 0     //RISC 
#define ARCH_SH4 0       //RISC 日立SuperH
#define ARCH_SPARC 0     //RISC Scalable Processor ARChitecture 1985年由太阳微系统所设计
#define ARCH_SPARC64 0   //RISC 
#define ARCH_TILEGX 0    //RISC TILE-Gx is a multicore processor family by Tilera.
#define ARCH_TILEPRO 0
#define ARCH_ALPHA 0     //RISC DEC公司Alpha处理器
#define ARCH_S390 0      //IBM ESA/390 (Enterprise Systems Architecture/390) was introduced in September 1990
#define ARCH_TOMI 0      //Venray的TOMI(Thread Optimized Multiprocessor)

//和CPU指令相关的宏：
#define HAVE_ARMV5TE 0
#define HAVE_ARMV5TE_INLINE 0
#define HAVE_ARMV5TE_EXTERNAL 0
#define HAVE_ARMV6 0
#define HAVE_ARMV6_INLINE 0
#define HAVE_ARMV6_EXTERNAL 0
#define HAVE_ARMV6T2 0
#define HAVE_ARMV6T2_INLINE 0
#define HAVE_ARMV6T2_EXTERNAL 0
#define HAVE_NEON 0               //ARM? NEON? 
#define HAVE_NEON_EXTERNAL 0
#define HAVE_NEON_INLINE 0
#define HAVE_VFP 0                //VFP（矢量浮点）提供低成本的单精度和倍精度浮点运算能力
#define HAVE_VFP_ARGS 0           //E:\libav\libavutil\arm\asm.S
#define HAVE_VFP_EXTERNAL 0
#define HAVE_VFP_INLINE 0
#define HAVE_VFPV3 0
#define HAVE_VFPV3_EXTERNAL 0
#define HAVE_VFPV3_INLINE 0
#define HAVE_ASM_MOD_Q 0          //libavutil\arm\intmath.h av_clipl_int32_arm
#define HAVE_ASM_MOD_Y 0          //libavcodec\arm\dca.h    int8x8_fmul_int32

#define HAVE_MMX 1
#define HAVE_MMX_EXTERNAL 1
#define HAVE_MMX_INLINE 0
#define HAVE_MMXEXT 1
#define HAVE_MMXEXT_EXTERNAL 1
#define HAVE_MMXEXT_INLINE 0
#define HAVE_AMD3DNOW 1
#define HAVE_AMD3DNOW_EXTERNAL 1
#define HAVE_AMD3DNOW_INLINE 0
#define HAVE_AMD3DNOWEXT 1
#define HAVE_AMD3DNOWEXT_EXTERNAL 1
#define HAVE_AMD3DNOWEXT_INLINE 0
#define HAVE_SSE 1
#define HAVE_SSE_EXTERNAL 1
#define HAVE_SSE_INLINE 0
#define HAVE_SSE2 1
#define HAVE_SSE2_EXTERNAL 1
#define HAVE_SSE2_INLINE 0
#define HAVE_SSE3 1
#define HAVE_SSE3_EXTERNAL 1
#define HAVE_SSE3_INLINE 0
#define HAVE_SSSE3 1
#define HAVE_SSSE3_EXTERNAL 1
#define HAVE_SSSE3_INLINE 0
#define HAVE_SSE4 1
#define HAVE_SSE4_EXTERNAL 1
#define HAVE_SSE4_INLINE 0
#define HAVE_SSE42 1
#define HAVE_SSE42_EXTERNAL 1
#define HAVE_SSE42_INLINE 0
#define HAVE_AVX 1
#define HAVE_AVX_EXTERNAL 1
#define HAVE_AVX_INLINE 0
#define HAVE_AVX2 1
#define HAVE_AVX2_EXTERNAL 1
#define HAVE_AVX2_INLINE 0
#define HAVE_FMA4 1
#define HAVE_FMA4_EXTERNAL 1
#define HAVE_FMA4_INLINE 0
#define HAVE_I686 1
#define HAVE_I686_EXTERNAL 0
#define HAVE_I686_INLINE 0

#define HAVE_ALTIVEC 0              //AltiVec是一个浮点和整型单指令流多数据流（SIMD）指令集
#define HAVE_ALTIVEC_EXTERNAL 0
#define HAVE_ALTIVEC_INLINE 0
#define HAVE_ALTIVEC_H 0            //#include <altivec.h> PowerPC AltiVec include file
#define HAVE_PPC4XX 0               //PowerPC Series (1992) 6xx 4xx 7xx 74xx 970 A2
#define HAVE_PPC4XX_EXTERNAL 0
#define HAVE_PPC4XX_INLINE 0
#define HAVE_DCBZL 0                //libavcodec\ppc\dsputil_ppc.c replace dcbz by dcbzl

#define HAVE_VIS 0                  //Visual Instruction Set, or VIS，是一个用于SPARC处理器的SIMD多媒体指令集扩展
#define HAVE_VIS_EXTERNAL 0
#define HAVE_VIS_INLINE 0

#define HAVE_LOONGSON 0             //libavcodec\mips\mathops.h

//数学函数定义：libavutil\libm.h
#define HAVE_ATANF 1
#define HAVE_ATAN2F 1
#define HAVE_CBRTF 1
#define HAVE_COSF 1
#define HAVE_EXP2 1
#define HAVE_EXP2F 1
#define HAVE_EXPF 1
#define HAVE_ISINF 1
#define HAVE_ISNAN 1
#define HAVE_LDEXPF 1
#define HAVE_LLRINT 1
#define HAVE_LLRINTF 1
#define HAVE_LOG2 0
#define HAVE_LOG2F 1
#define HAVE_LOG10F 1
#define HAVE_LRINT 1
#define HAVE_LRINTF 1
#define HAVE_POWF 1
#define HAVE_RINT 1
#define HAVE_ROUND 1
#define HAVE_ROUNDF 1
#define HAVE_SINF 1
#define HAVE_TRUNC 1
#define HAVE_TRUNCF 1

//第三方库：
#define HAVE_GSM_H 0        //libgsm.c
#define HAVE_LIBDC1394_1 0  //libavdevice\libdc1394.c control and capture streams from IEEE 1394 based cameras
#define HAVE_LIBDC1394_2 0
#define HAVE_SDL 0

//Windows系统API
#define HAVE_LIBC_MSVCRT 1
#define HAVE_COMMANDLINETOARGVW 1         //GetCommandLineW() cmdutils.c
#define HAVE_CRYPTGENRANDOM 1             //#include <wincrypt.h>  CryptAcquireContext
#define HAVE_DIRECT_H 1                   //#include <direct.h>
#define HAVE_IO_H 1                       //#include <io.h>
#define HAVE_DXVA_H 1                     //E:\libav\libavcodec\dxva2_internal.h  #include <dxva.h>
#define HAVE_DOS_PATHS 1                  //E:\libav\libavutil\avstring.c
#define HAVE_GETPROCESSAFFINITYMASK 1     //Retrieves the process affinity mask for the specified process and the system affinity mask for the system
#define HAVE_WINDOWS_H 1
#define HAVE_WINSOCK2_H 0
#define HAVE_GETADDRINFO 0                //if (getaddrinfo(hostname, NULL, &hints, &ai))
#define HAVE_GETPROCESSMEMORYINFO 1       //memcounters.PeakPagefileUsage
#define HAVE_GETPROCESSTIMES 1            //GetProcessTimes
#define HAVE_GETSYSTEMTIMEASFILETIME 1    //GetSystemTimeAsFileTime
#define HAVE_MAPVIEWOFFILE 1
#define HAVE_SETCONSOLETEXTATTRIBUTE 1    //SetConsoleTextAttribute
#define HAVE_SCHED_GETAFFINITY 0          //<sched.h> GetProcessAffinityMask
#define HAVE_VIRTUALALLOC 1               //VirtualAlloc

//UNIX系统API
#define HAVE_ALSA_ASOUNDLIB_H 0           //linux的主流音频体系结构 www.alsa-project.org
#define HAVE_ARPA_INET_H 0                //#include <arpa/inet.h> definitions for internet operations (UNIX)
#define HAVE_CDIO_PARANOIA_H 0            //GNU Compact Disc Input and Control Library (libcdio)
#define HAVE_CDIO_PARANOIA_PARANOIA_H 0   //GNU Compact Disc Input and Control Library (libcdio)
#define HAVE_FCNTL 0                      //fcntl(fd, F_SETFD, FD_CLOEXEC) manipulate file descriptor
#define HAVE_GETHRTIME 0                  //gethrtime, gethrvtime - get high resolution time
#define HAVE_GETOPT 0                     //#include "compat/getopt.c" the AT&T public domain source for getopt(3)
#define HAVE_GETRUSAGE 0                  //getrusage(RUSAGE_SELF, &rusage); get resource usage
#define HAVE_STRUCT_RUSAGE_RU_MAXRSS 0    //struct rusage rusage;
#define HAVE_GETSERVBYPORT 0              //getservbyport(sin->sin_port, flags & NI_DGRAM ? "udp" : "tcp");
#define HAVE_GETTIMEOFDAY 0               //gettimeofday(&tv, NULL); gettimeofday, settimeofday - get / set time
#define HAVE_INET_ATON 0                  //int inet_aton(const char *cp, struct in_addr *inp) 字符串IP地址转换为一个32位的网络序列IP地址
#define HAVE_JACK_PORT_GET_LATENCY_RANGE 0  //libavdevice\jack_audio.c
#define HAVE_SYS_MMAN_H 0                   //#include <sys/mman.h>
#define HAVE_SYS_PARAM_H 0                  //#include <sys/param.h>
#define HAVE_SYS_RESOURCE_H 0               //#include <sys/time.h> #include <sys/resource.h>
#define HAVE_SYS_SELECT_H 0                 //#include <sys/select.h>
#define HAVE_SYS_TIME_H 0                   //#include <sys/time.h>
#define HAVE_SYS_UN_H 0                     //The <sys/un.h> header shall define the sockaddr_un structure
#define HAVE_SYS_VIDEOIO_H 0                //#include <sys/videoio.h>
#define HAVE_SYS_SOUNDCARD_H 0              //#include <sys/soundcard.h>
#define HAVE_SOUNDCARD_H 0                  //#include <soundcard.h>
#define HAVE_SNDIO_H 0                      //<sndio.h> is the software layer of the OpenBSD operating system that manages the use of sound cards and MIDI ports.
#define HAVE_POLL_H 0                       //<poll.h> int poll(struct pollfd *fds, nfds_t nfds, int timeout); wait for some event on a file descriptor
#define HAVE_STRUCT_POLLFD 0                //==0 def struct pollfd
#define HAVE_DLFCN_H 0                      //<dlfcn.h> header defines at least the following macros for use in the construction of a dlopen()
#define HAVE_DLOPEN 0
#define HAVE_XLIB 0                         //Xlib (also known as libX11) is an X Window System protocol client library written in the C programming language.
#define HAVE_FORK 0                         //fork - create a child process
#define HAVE_MMAP 0                         //mmap, munmap - map or unmap files or devices into memory
#define HAVE_MPROTECT 0                     //#define USE_MMAP (HAVE_MMAP && HAVE_MPROTECT && defined MAP_ANONYMOUS)
#define HAVE_STRUCT_V4L2_FRMIVALENUM_DISCRETE 0 //Linux kernel: video 4 linux 2 supporting many USB webcams, TV tuners, and other devices
#define HAVE_VDPAU_X11 0                  //const HWAccel hwaccels[]

//设备驱动 libavdevice\bktr.c
//bktr -- Brooktree Bt848 / 849 / 878 / 879 and Pinnacle PCTV video capture driver
#define HAVE_DEV_BKTR_IOCTL_BT848_H 0
#define HAVE_DEV_BKTR_IOCTL_METEOR_H 0
#define HAVE_DEV_IC_BT8XX_H 0
#define HAVE_DEV_VIDEO_BKTR_IOCTL_BT848_H 0
#define HAVE_DEV_VIDEO_METEOR_IOCTL_METEOR_H 0
#define HAVE_MACHINE_IOCTL_BT848_H 0
#define HAVE_MACHINE_IOCTL_METEOR_H 0
#define HAVE_MACHINE_RW_BARRIER 0

//C Standard Library
#define HAVE_LOCALTIME_R 0  //strftime(buf, buf_size, s->text, &ltime)
#define HAVE_ISATTY 1       //int isatty(int desc); 函数：如果参数desc所代表的文件描述词为一终端机则返回1，否则返回0
#define HAVE_MALLOC_H 1     //#include <malloc.h>
#define HAVE_USLEEP 0       //usleep(usec);
#define HAVE_SETMODE 1      //setmode(fd, O_BINARY);
#define HAVE_SETRLIMIT 0    //setrlimit(RLIMIT_CPU, &rl
#define HAVE_SLEEP 1        //Sleep(usec / 1000);
#define HAVE_MKSTEMP 0      //<fcntl.h> mkstemp(*filename);
#define HAVE_RDTSC 1        //<intrin.h> #define AV_READ_TIME __rdtsc
#define HAVE_NANOSLEEP 0    //nanosleep(&ts, &ts)
#define HAVE_MM_EMPTY 1     //#include <mmintrin.h>  #define emms_c _mm_empty
#define HAVE_SYSCONF 0      //nb_cpus = sysconf(_SC_NPROC_ONLN);
#define HAVE_SYSCTL 0       //sysctl(mib, 2, &nb_cpus, &len, NULL, 0)
#define HAVE_STRERROR_R 0   //ret = strerror_r(AVUNERROR(errnum), errbuf, errbuf_size);
#define HAVE_STRPTIME 0     //ret1 = strptime(datestr, "%Y - %m - %d %T", &time1);
#define HAVE_UNISTD_H 0     //#include <unistd.h>
#define HAVE_CLOSESOCKET 0  //== 0 #define closesocket close

//IPv6支持，网络数据结构
#define HAVE_STRUCT_IP_MREQ_SOURCE 0
#define HAVE_STRUCT_IPV6_MREQ 0
#define HAVE_STRUCT_SOCKADDR_IN6 0
#define HAVE_STRUCT_SOCKADDR_SA_LEN 0  //uint8_t ss_len; + uint8_t ss_family;  == uint16_t ss_family;
#define HAVE_STRUCT_SOCKADDR_STORAGE 0 //==0 需要定义 struct sockaddr_storage
#define HAVE_STRUCT_GROUP_SOURCE_REQ 0 //libavformat\udp.c
#define HAVE_SOCKLEN_T 0
#define HAVE_STRUCT_ADDRINFO 0         //libavformat\network.h

//线程，字节序，
#define HAVE_THREADS 1
#define HAVE_W32THREADS 1
#define HAVE_PTHREADS 0
#define HAVE_BIGENDIAN 0

//内存对齐方式：libavutil\mem.c
#define HAVE_ALIGNED_MALLOC 1   //_aligned_malloc(size, 32); _aligned_realloc(ptr, size, 32); _aligned_free(ptr);
#define HAVE_ALIGNED_STACK 0
#define HAVE_POSIX_MEMALIGN 0
#define HAVE_MEMALIGN 0
#define HAVE_LOCAL_ALIGNED_16 1 //libavutil\internal.h
#define HAVE_LOCAL_ALIGNED_8 1

//和汇编，指令相关的宏
#define HAVE_GNU_AS 1
#define HAVE_IBM_ASM 0           //ppc\asm.S
#define HAVE_YASM 1
#define HAVE_INLINE_ASM 0
#define HAVE_SYMVER 0            //.symver汇编宏指令
#define HAVE_SYMVER_ASM_LABEL 0
#define HAVE_SYMVER_GNU_ASM 0
#define HAVE_CPUNOP 1            //In computer science, a NOP or NOOP (short for No Operation) is an assembly language instruction
#define HAVE_EBP_AVAILABLE 0     //libavutil\x86\asm.h 
#define HAVE_EBX_AVAILABLE 0     //libavutil\x86\asm.h
#define HAVE_INLINE_ASM_LABELS 0 //libavutil\ppc\timer.h read_time(void) from section 2.2.1 of the 32-bit PowerPC PEM
#define HAVE_LDBRX 0             //ldbrx __asm__ ("ldbrx   %0, %y1" : "=r"(v) : "Z"(*(const uint64_t*)p));
#define HAVE_XFORM_ASM 0         //libavutil\ppc\intreadwrite.h  指令：lhbrx，sthbrx，lwbrx，stwbrx，ldbrx，stdbrx，lwbrx，stwbrx
#define HAVE_XMM_CLOBBERS 0      //mark XMM registers as clobbered  in order to tell GCC that your code is using some of the processor's registers
#define HAVE_FAST_UNALIGNED 1    //unaligned memory accesses
#define HAVE_FAST_64BIT 0
#define HAVE_FAST_CLZ 1          //#define ff_ctz(v) __builtin_ctz(v)
#define HAVE_FAST_CMOV 0         //x86 CMOV instruction

//原子操作定义：libavutil\atomic.h 
#define HAVE_ATOMICS_NATIVE 1
#define HAVE_ATOMICS_WIN32 1
#define HAVE_ATOMICS_GCC 0
#define HAVE_ATOMICS_SUNCC 0
#define HAVE_ATOMIC_CAS_PTR 0
#define HAVE_SYNC_VAL_COMPARE_AND_SWAP 0
#define HAVE_MEMORYBARRIER 1     //内存屏障 memory barrier 乱序执行

//其它
#define HAVE_ATTRIBUTE_MAY_ALIAS 0
#define HAVE_ATTRIBUTE_PACKED 0
#define HAVE_PRAGMA_DEPRECATED 1

//杂项
#define CONFIG_DOC 1
#define CONFIG_NONFREE 0
#define CONFIG_GPL 0
#define CONFIG_GPLV3 0
#define CONFIG_LGPLV3 0
#define CONFIG_VERSION3 0
#define CONFIG_MEMALIGN_HACK 0
#define CONFIG_SHARED 0
#define CONFIG_STATIC 1
#define CONFIG_TEXI2HTML 0
#define CONFIG_RUNTIME_CPUDETECT 1
//算法
#define CONFIG_DCT 0         //+= dct.o dct32_fixed.o dct32_float.o            Discrete Cosine Transform
#define CONFIG_MDCT 1        //modified discrete cosine transform
#define CONFIG_AANDCTTABLES 1 //+= aandcttab.o AAN (Arai Agui Aakajima) (I)DCT tables
#define CONFIG_FFT 1         //avfft.o fft_fixed.o fft_float.o $(FFT-OBJS-yes)  Fast Fourier Transform
#define CONFIG_RDFT 0        //Real discrete Fourier transform (real FFT)
#define CONFIG_LZO 0         //Lempel–Ziv–Oberhumer (LZO)
#define CONFIG_HUFFMAN 0     //Huffman coding
#define CONFIG_GOLOMB 1      //+= golomb.o
                             //exp golomb vlc stuff (Exp-Golomb coding for k = 0 is used in the H.264/MPEG-4 AVC video compression standard)
#define CONFIG_SINEWIN 1     //sine window
#define CONFIG_RANGECODER 0  //Range encoding += rangecoder.o
//DSP
#define CONFIG_HPELDSP 1
#define CONFIG_DSPUTIL 1      //+= dsputil.o faanidct.o simple_idct.o jrevdct.o
#define CONFIG_H263DSP 1      //+= h263dsp.o
#define CONFIG_H264DSP 1      //+= h264dsp.o h264idct.o
#define CONFIG_H264CHROMA 1        //+= h264chroma.o
#define CONFIG_H264PRED 1          //+= h264pred.o  H.264 / AVC / MPEG4 part10 prediction functions
#define CONFIG_H264QPEL 1          //+= h264qpel.o Quarter-pixel motion (also known as Q-pel motion or Qpel motion)
#define CONFIG_VIDEODSP 1
#define CONFIG_VP3DSP 0
#define CONFIG_AC3DSP 0       //+= ac3dsp.o
#define CONFIG_MPEGAUDIODSP 0
//测试
#define CONFIG_NEON_CLOBBER_TEST 0
#define CONFIG_XMM_CLOBBER_TEST 0
//Microchip（微芯公司）的PIC单片机系列
#define CONFIG_PIC 0              //PIC microcontroller
#define CONFIG_SRAM 0             //allow use of on-chip SRAM on some systems, Static Random-Access Memory, SRAM
//libavcodec makefile
#define CONFIG_SMALL 0             //$(SUBDIR)%_tablegen$(HOSTEXESUF): HOSTCFLAGS += -DCONFIG_SMALL=1
#define CONFIG_AUDIO_FRAME_QUEUE 1 //+= audio_frame_queue.o  Audio Frame Queue
#define CONFIG_INTRAX8 1           //+= intrax8.o intrax8dsp.o
#define CONFIG_LPC 0               //+= lpc.o  LPCContext
#define CONFIG_MPEGAUDIO 0         //+= mpegaudio.o mpegaudiodata.o mpegaudiodecheader.o
#define CONFIG_MPEGVIDEO 1         //+= mpegvideo.o mpegvideo_motion.o
#define CONFIG_MPEGVIDEOENC 1      //+= mpegvideo_enc.o mpeg12data.o motion_est.o ratecontrol.o
#define CONFIG_ERROR_RESILIENCE 1      //+= error_resilience.o  ERContext
#define CONFIG_LSP 0                   //LSP routines for ACELP-based codecs
#define CONFIG_HARDCODED_TABLES 0      //+= cos_tables.o cos_fixed_tables.o
//libavformat makefile
#define CONFIG_NETWORK 0
#define CONFIG_RIFFDEC 1               //+= riffdec.o
#define CONFIG_RIFFENC 1               //+= riffenc.o
#define CONFIG_RTPENC_CHAIN 1          //+= rtpenc_chain.o rtp.o
#define CONFIG_RTPDEC 0                //rtp protocal
//
#define CONFIG_GRAY 0
#define CONFIG_POD2MAN 0               //pod2man, a great way to write Unix man pages
#define CONFIG_GCRYPT 0                //<gcrypt.h>
#define CONFIG_NETTLE 0                //<gmp.h>, <nettle/bignum.h>
#define CONFIG_THUMB 0                 //Thumb指令可以看做是ARM指令压缩形式的子集
#define CONFIG_SWSCALE_ALPHA 1         //AV_PIX_FMT_YUVA420P
#define CONFIG_SAFE_BITSTREAM_READER 1 //Safe bitstream reading Bitstream reader API docs
//指定编译的库及命令
#define CONFIG_AVCODEC 1
#define CONFIG_AVDEVICE 0
#define CONFIG_AVFILTER 0
#define CONFIG_AVFORMAT 1
#define CONFIG_AVRESAMPLE 1
#define CONFIG_AVUTIL 1
#define CONFIG_SWSCALE 1
#define CONFIG_AVCONV 0
#define CONFIG_AVPLAY 0
#define CONFIG_AVPROBE 0
#define CONFIG_AVSERVER 0
#define CONFIG_AVCODEC_EXAMPLE 1
#define CONFIG_METADATA_EXAMPLE 1
#define CONFIG_OUTPUT_EXAMPLE 1
#define CONFIG_TRANSCODE_AAC_EXAMPLE 1
//BSFS、DEVS、CODER、FILTER、PARSER、PROTOCOL
#define CONFIG_BSFS 0
#define CONFIG_OUTDEVS 0
#define CONFIG_INDEVS 0
#define CONFIG_DECODERS 1
#define CONFIG_ENCODERS 1  //+= faandct.o jfdctfst.o jfdctint.o
#define CONFIG_DEMUXERS 1
#define CONFIG_MUXERS 1
#define CONFIG_FILTERS 0
#define CONFIG_HWACCELS 0
#define CONFIG_PARSERS 1
#define CONFIG_PROTOCOLS 1
//bit stream filter ???
#define CONFIG_AAC_ADTSTOASC_BSF 0
#define CONFIG_CHOMP_BSF 0
#define CONFIG_DUMP_EXTRADATA_BSF 0
#define CONFIG_H264_MP4TOANNEXB_BSF 0
#define CONFIG_IMX_DUMP_HEADER_BSF 0
#define CONFIG_MJPEG2JPEG_BSF 0
#define CONFIG_MJPEGA_DUMP_HEADER_BSF 0
#define CONFIG_MOV2TEXTSUB_BSF 0
#define CONFIG_NOISE_BSF 0
#define CONFIG_REMOVE_EXTRADATA_BSF 0
#define CONFIG_TEXT2MOVSUB_BSF 0
//硬件加速
#define CONFIG_VDA 0    //Video Decode Acceleration Framework Reference - MAC & IOS
#define CONFIG_H264_VDA_HWACCEL 0
#define CONFIG_VAAPI 0  //Linux: Video Acceleration API (VA API)  cross-platform API
#define CONFIG_H263_VAAPI_HWACCEL 0
#define CONFIG_H264_VAAPI_HWACCEL 0
#define CONFIG_MPEG2_VAAPI_HWACCEL 0
#define CONFIG_MPEG4_VAAPI_HWACCEL 0
#define CONFIG_WMV3_VAAPI_HWACCEL 0
#define CONFIG_VC1_VAAPI_HWACCEL 0
#define CONFIG_VDPAU 0  //Linux:(NVIDIA) Video Decode and Presentation API for Unix
#define CONFIG_H263_VDPAU_HWACCEL 0
#define CONFIG_H264_VDPAU_HWACCEL 0
#define CONFIG_MPEG1_VDPAU_HWACCEL 0
#define CONFIG_MPEG2_VDPAU_HWACCEL 0
#define CONFIG_MPEG4_VDPAU_HWACCEL 0
#define CONFIG_VC1_VDPAU_HWACCEL 0
#define CONFIG_WMV3_VDPAU_HWACCEL 0
#define CONFIG_DXVA2 1  //+= dxva2.o  DXVA 2	Windows Vista	EVR (DirectShow and Media Foundation)
#define CONFIG_H264_DXVA2_HWACCEL 0
#define CONFIG_MPEG2_DXVA2_HWACCEL 0
#define CONFIG_VC1_DXVA2_HWACCEL 0
#define CONFIG_WMV3_DXVA2_HWACCEL 0
//输出设备
#define CONFIG_ALSA_OUTDEV 0
#define CONFIG_OSS_OUTDEV 0
#define CONFIG_SNDIO_OUTDEV 0
//输入设备
#define CONFIG_ALSA_INDEV 0
#define CONFIG_BKTR_INDEV 0
#define CONFIG_DV1394_INDEV 0
#define CONFIG_FBDEV_INDEV 0
#define CONFIG_JACK_INDEV 0
#define CONFIG_OSS_INDEV 0
#define CONFIG_PULSE_INDEV 0
#define CONFIG_SNDIO_INDEV 0
#define CONFIG_V4L2_INDEV 0
#define CONFIG_VFWCAP_INDEV 0
#define CONFIG_X11GRAB_INDEV 0
#define CONFIG_LIBCDIO_INDEV 0
#define CONFIG_LIBDC1394_INDEV 0
//第三方库
#define CONFIG_AVISYNTH 0
#define CONFIG_BZLIB 0
#define CONFIG_FREI0R 0
#define CONFIG_GNUTLS 0
#define CONFIG_LIBCDIO 0
#define CONFIG_LIBDC1394 0
#define CONFIG_LIBFAAC 0
#define CONFIG_LIBFDK_AAC 0
#define CONFIG_LIBFREETYPE 0
#define CONFIG_LIBGSM 0
#define CONFIG_LIBILBC 0
#define CONFIG_LIBMP3LAME 0
#define CONFIG_LIBOPENCORE_AMRNB 0
#define CONFIG_LIBOPENCORE_AMRWB 0
#define CONFIG_LIBOPENCV 0
#define CONFIG_LIBOPENJPEG 0
#define CONFIG_LIBOPUS 0
#define CONFIG_LIBPULSE 0
#define CONFIG_LIBRTMP 0
#define CONFIG_LIBSCHROEDINGER 0
#define CONFIG_LIBSPEEX 0
#define CONFIG_LIBTHEORA 0
#define CONFIG_LIBVO_AACENC 0
#define CONFIG_LIBVO_AMRWBENC 0
#define CONFIG_LIBVORBIS 0
#define CONFIG_LIBVPX 0
#define CONFIG_LIBWAVPACK 0
#define CONFIG_LIBWEBP 0
#define CONFIG_LIBX264 0
#define CONFIG_LIBXAVS 0
#define CONFIG_LIBXVID 0
#define CONFIG_OPENSSL 0
#define CONFIG_X11GRAB 0
#define CONFIG_ZLIB 0
//
#define CONFIG_H263_PARSER 1
#define CONFIG_MPEG4VIDEO_PARSER 1
#define CONFIG_AAC_PARSER 0
#define CONFIG_AAC_LATM_PARSER 0
#define CONFIG_AC3_PARSER 0
#define CONFIG_ADX_PARSER 0
#define CONFIG_CAVSVIDEO_PARSER 0
#define CONFIG_COOK_PARSER 0
#define CONFIG_DCA_PARSER 0
#define CONFIG_DIRAC_PARSER 0
#define CONFIG_DNXHD_PARSER 0
#define CONFIG_DVBSUB_PARSER 0
#define CONFIG_DVDSUB_PARSER 0
#define CONFIG_FLAC_PARSER 0
#define CONFIG_GSM_PARSER 0
#define CONFIG_H261_PARSER 0
#define CONFIG_H264_PARSER 0
#define CONFIG_HEVC_PARSER 0
#define CONFIG_MJPEG_PARSER 0
#define CONFIG_MLP_PARSER 0
#define CONFIG_MPEGAUDIO_PARSER 0
#define CONFIG_MPEGVIDEO_PARSER 0
#define CONFIG_PNG_PARSER 0
#define CONFIG_PNM_PARSER 0
#define CONFIG_RV30_PARSER 0
#define CONFIG_RV40_PARSER 0
#define CONFIG_TAK_PARSER 0
#define CONFIG_VC1_PARSER 0
#define CONFIG_VORBIS_PARSER 0
#define CONFIG_VP3_PARSER 0
#define CONFIG_VP8_PARSER 0
//输出入文件协议
#define CONFIG_FILE_PROTOCOL 1
#define CONFIG_CONCAT_PROTOCOL 0
#define CONFIG_CRYPTO_PROTOCOL 0
#define CONFIG_FFRTMPCRYPT_PROTOCOL 0
#define CONFIG_FFRTMPHTTP_PROTOCOL 0
#define CONFIG_FILE_PROTOCOL 1
#define CONFIG_GOPHER_PROTOCOL 0
#define CONFIG_HLS_PROTOCOL 0
#define CONFIG_HTTP_PROTOCOL 0
#define CONFIG_HTTPPROXY_PROTOCOL 0
#define CONFIG_HTTPS_PROTOCOL 0
#define CONFIG_MMSH_PROTOCOL 0
#define CONFIG_MMST_PROTOCOL 0
#define CONFIG_MD5_PROTOCOL 0
#define CONFIG_PIPE_PROTOCOL 0
#define CONFIG_RTMP_PROTOCOL 0
#define CONFIG_RTMPE_PROTOCOL 0
#define CONFIG_RTMPS_PROTOCOL 0
#define CONFIG_RTMPT_PROTOCOL 0
#define CONFIG_RTMPTE_PROTOCOL 0
#define CONFIG_RTMPTS_PROTOCOL 0
#define CONFIG_RTP_PROTOCOL 0
#define CONFIG_SCTP_PROTOCOL 0
#define CONFIG_SRTP_PROTOCOL 0
#define CONFIG_TCP_PROTOCOL 0
#define CONFIG_TLS_PROTOCOL 0
#define CONFIG_UDP_PROTOCOL 0
#define CONFIG_UNIX_PROTOCOL 0
#define CONFIG_LIBRTMP_PROTOCOL 0
#define CONFIG_LIBRTMPE_PROTOCOL 0
#define CONFIG_LIBRTMPS_PROTOCOL 0
#define CONFIG_LIBRTMPT_PROTOCOL 0
#define CONFIG_LIBRTMPTE_PROTOCOL 0
//滤镜
#define CONFIG_AFORMAT_FILTER 0
#define CONFIG_AMIX_FILTER 0
#define CONFIG_ANULL_FILTER 0
#define CONFIG_ASETPTS_FILTER 0
#define CONFIG_ASHOWINFO_FILTER 0
#define CONFIG_ASPLIT_FILTER 0
#define CONFIG_ASYNCTS_FILTER 0
#define CONFIG_ATRIM_FILTER 0
#define CONFIG_CHANNELMAP_FILTER 0
#define CONFIG_CHANNELSPLIT_FILTER 0
#define CONFIG_COMPAND_FILTER 0
#define CONFIG_JOIN_FILTER 0
#define CONFIG_RESAMPLE_FILTER 0
#define CONFIG_VOLUME_FILTER 0
#define CONFIG_ANULLSRC_FILTER 0
#define CONFIG_ANULLSINK_FILTER 0
#define CONFIG_BLACKFRAME_FILTER 0
#define CONFIG_BOXBLUR_FILTER 0
#define CONFIG_COPY_FILTER 0
#define CONFIG_CROP_FILTER 0
#define CONFIG_CROPDETECT_FILTER 0
#define CONFIG_DELOGO_FILTER 0
#define CONFIG_DRAWBOX_FILTER 0
#define CONFIG_DRAWTEXT_FILTER 0
#define CONFIG_FADE_FILTER 0
#define CONFIG_FIELDORDER_FILTER 0
#define CONFIG_FORMAT_FILTER 0
#define CONFIG_FPS_FILTER 0
#define CONFIG_FRAMEPACK_FILTER 0
#define CONFIG_FREI0R_FILTER 0
#define CONFIG_GRADFUN_FILTER 0
#define CONFIG_HFLIP_FILTER 0
#define CONFIG_HQDN3D_FILTER 0
#define CONFIG_INTERLACE_FILTER 0
#define CONFIG_LUT_FILTER 0
#define CONFIG_LUTRGB_FILTER 0
#define CONFIG_LUTYUV_FILTER 0
#define CONFIG_NEGATE_FILTER 0
#define CONFIG_NOFORMAT_FILTER 0
#define CONFIG_NULL_FILTER 0
#define CONFIG_OCV_FILTER 0
#define CONFIG_OVERLAY_FILTER 0
#define CONFIG_PAD_FILTER 0
#define CONFIG_PIXDESCTEST_FILTER 0
#define CONFIG_SCALE_FILTER 0
#define CONFIG_SELECT_FILTER 0
#define CONFIG_SETDAR_FILTER 0
#define CONFIG_SETPTS_FILTER 0
#define CONFIG_SETSAR_FILTER 0
#define CONFIG_SETTB_FILTER 0
#define CONFIG_SHOWINFO_FILTER 0
#define CONFIG_SPLIT_FILTER 0
#define CONFIG_TRANSPOSE_FILTER 0
#define CONFIG_TRIM_FILTER 0
#define CONFIG_UNSHARP_FILTER 0
#define CONFIG_VFLIP_FILTER 0
#define CONFIG_YADIF_FILTER 0
#define CONFIG_COLOR_FILTER 0
#define CONFIG_FREI0R_SRC_FILTER 0
#define CONFIG_MOVIE_FILTER 0
#define CONFIG_NULLSRC_FILTER 0
#define CONFIG_RGBTESTSRC_FILTER 0
#define CONFIG_TESTSRC_FILTER 0
#define CONFIG_NULLSINK_FILTER 0
//解码器
#define CONFIG_FLV_DECODER 1
#define CONFIG_H261_DECODER 1
#define CONFIG_H263_DECODER 1
#define CONFIG_MPEG4_DECODER 1
#define CONFIG_WMV2_DECODER 1
#define CONFIG_MPEG1VIDEO_DECODER 1
#define CONFIG_MPEG2VIDEO_DECODER 1
#define CONFIG_MSMPEG4V2_DECODER 1
#define CONFIG_MSMPEG4V3_DECODER 1
#define CONFIG_MJPEG_DECODER 1
#define CONFIG_RV10_DECODER 1
#define CONFIG_RV20_DECODER 1
#define CONFIG_JPEGLS_DECODER 1
#define CONFIG_MSMPEG4V1_DECODER 1
#define CONFIG_H263I_DECODER 1
#define CONFIG_H264_DECODER 1
#define CONFIG_SVQ3_DECODER 1
#define CONFIG_AAC_DECODER 1
#define CONFIG_AASC_DECODER 0
#define CONFIG_AIC_DECODER 0
#define CONFIG_AMV_DECODER 0
#define CONFIG_ANM_DECODER 0
#define CONFIG_ANSI_DECODER 0
#define CONFIG_ASV1_DECODER 0
#define CONFIG_ASV2_DECODER 0
#define CONFIG_AURA_DECODER 0
#define CONFIG_AURA2_DECODER 0
#define CONFIG_AVS_DECODER 0
#define CONFIG_BETHSOFTVID_DECODER 0
#define CONFIG_BFI_DECODER 0
#define CONFIG_BINK_DECODER 0
#define CONFIG_BMP_DECODER 0
#define CONFIG_BMV_VIDEO_DECODER 0
#define CONFIG_C93_DECODER 0
#define CONFIG_CAVS_DECODER 0
#define CONFIG_CDGRAPHICS_DECODER 0
#define CONFIG_CDXL_DECODER 0
#define CONFIG_CINEPAK_DECODER 0
#define CONFIG_CLJR_DECODER 0
#define CONFIG_CLLC_DECODER 0
#define CONFIG_COMFORTNOISE_DECODER 0
#define CONFIG_CSCD_DECODER 0
#define CONFIG_CYUV_DECODER 0
#define CONFIG_DFA_DECODER 0
#define CONFIG_DNXHD_DECODER 0
#define CONFIG_DPX_DECODER 0
#define CONFIG_DSICINVIDEO_DECODER 0
#define CONFIG_DVVIDEO_DECODER 0
#define CONFIG_DXA_DECODER 0
#define CONFIG_DXTORY_DECODER 0
#define CONFIG_EACMV_DECODER 0
#define CONFIG_EAMAD_DECODER 0
#define CONFIG_EATGQ_DECODER 0
#define CONFIG_EATGV_DECODER 0
#define CONFIG_EATQI_DECODER 0
#define CONFIG_EIGHTBPS_DECODER 0
#define CONFIG_EIGHTSVX_EXP_DECODER 0
#define CONFIG_EIGHTSVX_FIB_DECODER 0
#define CONFIG_ESCAPE124_DECODER 0
#define CONFIG_ESCAPE130_DECODER 0
#define CONFIG_FFV1_DECODER 0
#define CONFIG_FFVHUFF_DECODER 0
#define CONFIG_FIC_DECODER 0
#define CONFIG_FLASHSV_DECODER 0
#define CONFIG_FLASHSV2_DECODER 0
#define CONFIG_FLIC_DECODER 0
#define CONFIG_FLV_DECODER 1
#define CONFIG_FOURXM_DECODER 0
#define CONFIG_FRAPS_DECODER 0
#define CONFIG_FRWU_DECODER 0
#define CONFIG_G2M_DECODER 0
#define CONFIG_GIF_DECODER 0
#define CONFIG_H261_DECODER 1
#define CONFIG_H263_DECODER 1
#define CONFIG_H263I_DECODER 1
#define CONFIG_H264_DECODER 1
#define CONFIG_HEVC_DECODER 0
#define CONFIG_HNM4_VIDEO_DECODER 0
#define CONFIG_HUFFYUV_DECODER 0
#define CONFIG_IDCIN_DECODER 0
#define CONFIG_IFF_BYTERUN1_DECODER 0
#define CONFIG_IFF_ILBM_DECODER 0
#define CONFIG_INDEO2_DECODER 0
#define CONFIG_INDEO3_DECODER 0
#define CONFIG_INDEO4_DECODER 0
#define CONFIG_INDEO5_DECODER 0
#define CONFIG_INTERPLAY_VIDEO_DECODER 0
#define CONFIG_JPEG2000_DECODER 0
#define CONFIG_JPEGLS_DECODER 1
#define CONFIG_JV_DECODER 0
#define CONFIG_KGV1_DECODER 0
#define CONFIG_KMVC_DECODER 0
#define CONFIG_LAGARITH_DECODER 0
#define CONFIG_LOCO_DECODER 0
#define CONFIG_MDEC_DECODER 0
#define CONFIG_MIMIC_DECODER 0
#define CONFIG_MJPEG_DECODER 1
#define CONFIG_MJPEGB_DECODER 0
#define CONFIG_MMVIDEO_DECODER 0
#define CONFIG_MOTIONPIXELS_DECODER 0
#define CONFIG_MPEG_XVMC_DECODER 0
#define CONFIG_MPEG1VIDEO_DECODER 1
#define CONFIG_MPEG2VIDEO_DECODER 1
#define CONFIG_MPEG4_DECODER 1
#define CONFIG_MSA1_DECODER 0
#define CONFIG_MSMPEG4V1_DECODER 1
#define CONFIG_MSMPEG4V2_DECODER 1
#define CONFIG_MSMPEG4V3_DECODER 1
#define CONFIG_MSRLE_DECODER 0
#define CONFIG_MSS1_DECODER 0
#define CONFIG_MSS2_DECODER 0
#define CONFIG_MSVIDEO1_DECODER 0
#define CONFIG_MSZH_DECODER 0
#define CONFIG_MTS2_DECODER 0
#define CONFIG_MXPEG_DECODER 0
#define CONFIG_NUV_DECODER 0
#define CONFIG_PAM_DECODER 0
#define CONFIG_PBM_DECODER 0
#define CONFIG_PCX_DECODER 0
#define CONFIG_PGM_DECODER 0
#define CONFIG_PGMYUV_DECODER 0
#define CONFIG_PICTOR_DECODER 0
#define CONFIG_PNG_DECODER 0
#define CONFIG_PPM_DECODER 0
#define CONFIG_PRORES_DECODER 0
#define CONFIG_PTX_DECODER 0
#define CONFIG_QDRAW_DECODER 0
#define CONFIG_QPEG_DECODER 0
#define CONFIG_QTRLE_DECODER 0
#define CONFIG_R10K_DECODER 0
#define CONFIG_R210_DECODER 0
#define CONFIG_RAWVIDEO_DECODER 0
#define CONFIG_RL2_DECODER 0
#define CONFIG_ROQ_DECODER 0
#define CONFIG_RPZA_DECODER 0
#define CONFIG_RV10_DECODER 1
#define CONFIG_RV20_DECODER 1
#define CONFIG_RV30_DECODER 0
#define CONFIG_RV40_DECODER 0
#define CONFIG_S302M_DECODER 0
#define CONFIG_SGI_DECODER 0
#define CONFIG_SMACKER_DECODER 0
#define CONFIG_SMC_DECODER 0
#define CONFIG_SP5X_DECODER 0
#define CONFIG_SUNRAST_DECODER 0
#define CONFIG_SVQ1_DECODER 0
#define CONFIG_SVQ3_DECODER 1
#define CONFIG_TARGA_DECODER 0
#define CONFIG_THEORA_DECODER 0
#define CONFIG_THP_DECODER 0
#define CONFIG_TIERTEXSEQVIDEO_DECODER 0
#define CONFIG_TIFF_DECODER 0
#define CONFIG_TMV_DECODER 0
#define CONFIG_TRUEMOTION1_DECODER 0
#define CONFIG_TRUEMOTION2_DECODER 0
#define CONFIG_TSCC_DECODER 0
#define CONFIG_TSCC2_DECODER 0
#define CONFIG_TXD_DECODER 0
#define CONFIG_ULTI_DECODER 0
#define CONFIG_UTVIDEO_DECODER 0
#define CONFIG_V210_DECODER 0
#define CONFIG_V210X_DECODER 0
#define CONFIG_V410_DECODER 0
#define CONFIG_VB_DECODER 0
#define CONFIG_VBLE_DECODER 0
#define CONFIG_VC1_DECODER 0
#define CONFIG_VC1IMAGE_DECODER 0
#define CONFIG_VCR1_DECODER 0
#define CONFIG_VMDVIDEO_DECODER 0
#define CONFIG_VMNC_DECODER 0
#define CONFIG_VP3_DECODER 0
#define CONFIG_VP5_DECODER 0
#define CONFIG_VP6_DECODER 0
#define CONFIG_VP6A_DECODER 0
#define CONFIG_VP6F_DECODER 0
#define CONFIG_VP8_DECODER 0
#define CONFIG_VP9_DECODER 0
#define CONFIG_VQA_DECODER 0
#define CONFIG_WEBP_DECODER 0
#define CONFIG_WMV1_DECODER 0
#define CONFIG_WMV2_DECODER 1
#define CONFIG_WMV3_DECODER 0
#define CONFIG_WMV3IMAGE_DECODER 0
#define CONFIG_WNV1_DECODER 0
#define CONFIG_XAN_WC3_DECODER 0
#define CONFIG_XAN_WC4_DECODER 0
#define CONFIG_XL_DECODER 0
#define CONFIG_XWD_DECODER 0
#define CONFIG_YOP_DECODER 0
#define CONFIG_ZEROCODEC_DECODER 0
#define CONFIG_ZLIB_DECODER 0
#define CONFIG_ZMBV_DECODER 0
#define CONFIG_AAC_DECODER 1
#define CONFIG_AAC_LATM_DECODER 0
#define CONFIG_AC3_DECODER 0
#define CONFIG_ALAC_DECODER 0
#define CONFIG_ALS_DECODER 0
#define CONFIG_AMRNB_DECODER 0
#define CONFIG_AMRWB_DECODER 0
#define CONFIG_APE_DECODER 0
#define CONFIG_ATRAC1_DECODER 0
#define CONFIG_ATRAC3_DECODER 0
#define CONFIG_ATRAC3P_DECODER 0
#define CONFIG_BINKAUDIO_DCT_DECODER 0
#define CONFIG_BINKAUDIO_RDFT_DECODER 0
#define CONFIG_BMV_AUDIO_DECODER 0
#define CONFIG_COOK_DECODER 0
#define CONFIG_DCA_DECODER 0
#define CONFIG_DSICINAUDIO_DECODER 0
#define CONFIG_EAC3_DECODER 0
#define CONFIG_FLAC_DECODER 0
#define CONFIG_G723_1_DECODER 0
#define CONFIG_GSM_DECODER 0
#define CONFIG_GSM_MS_DECODER 0
#define CONFIG_IAC_DECODER 0
#define CONFIG_IMC_DECODER 0
#define CONFIG_MACE3_DECODER 0
#define CONFIG_MACE6_DECODER 0
#define CONFIG_METASOUND_DECODER 0
#define CONFIG_MLP_DECODER 0
#define CONFIG_MP1_DECODER 0
#define CONFIG_MP1FLOAT_DECODER 0
#define CONFIG_MP2_DECODER 0
#define CONFIG_MP2FLOAT_DECODER 0
#define CONFIG_MP3_DECODER 0
#define CONFIG_MP3FLOAT_DECODER 0
#define CONFIG_MP3ADU_DECODER 0
#define CONFIG_MP3ADUFLOAT_DECODER 0
#define CONFIG_MP3ON4_DECODER 0
#define CONFIG_MP3ON4FLOAT_DECODER 0
#define CONFIG_MPC7_DECODER 0
#define CONFIG_MPC8_DECODER 0
#define CONFIG_NELLYMOSER_DECODER 0
#define CONFIG_QCELP_DECODER 0
#define CONFIG_QDM2_DECODER 0
#define CONFIG_RA_144_DECODER 0
#define CONFIG_RA_288_DECODER 0
#define CONFIG_RALF_DECODER 0
#define CONFIG_SHORTEN_DECODER 0
#define CONFIG_SIPR_DECODER 0
#define CONFIG_SMACKAUD_DECODER 0
#define CONFIG_TAK_DECODER 0
#define CONFIG_TRUEHD_DECODER 0
#define CONFIG_TRUESPEECH_DECODER 0
#define CONFIG_TTA_DECODER 0
#define CONFIG_TWINVQ_DECODER 0
#define CONFIG_VMDAUDIO_DECODER 0
#define CONFIG_VORBIS_DECODER 0
#define CONFIG_WAVPACK_DECODER 0
#define CONFIG_WMALOSSLESS_DECODER 0
#define CONFIG_WMAPRO_DECODER 0
#define CONFIG_WMAV1_DECODER 0
#define CONFIG_WMAV2_DECODER 0
#define CONFIG_WMAVOICE_DECODER 0
#define CONFIG_WS_SND1_DECODER 0
#define CONFIG_PCM_ALAW_DECODER 0
#define CONFIG_PCM_BLURAY_DECODER 0
#define CONFIG_PCM_DVD_DECODER 0
#define CONFIG_PCM_F32BE_DECODER 0
#define CONFIG_PCM_F32LE_DECODER 0
#define CONFIG_PCM_F64BE_DECODER 0
#define CONFIG_PCM_F64LE_DECODER 0
#define CONFIG_PCM_LXF_DECODER 0
#define CONFIG_PCM_MULAW_DECODER 0
#define CONFIG_PCM_S8_DECODER 0
#define CONFIG_PCM_S8_PLANAR_DECODER 0
#define CONFIG_PCM_S16BE_DECODER 0
#define CONFIG_PCM_S16LE_DECODER 0
#define CONFIG_PCM_S16LE_PLANAR_DECODER 0
#define CONFIG_PCM_S24BE_DECODER 0
#define CONFIG_PCM_S24DAUD_DECODER 0
#define CONFIG_PCM_S24LE_DECODER 0
#define CONFIG_PCM_S24LE_PLANAR_DECODER 0
#define CONFIG_PCM_S32BE_DECODER 0
#define CONFIG_PCM_S32LE_DECODER 0
#define CONFIG_PCM_S32LE_PLANAR_DECODER 0
#define CONFIG_PCM_U8_DECODER 0
#define CONFIG_PCM_U16BE_DECODER 0
#define CONFIG_PCM_U16LE_DECODER 0
#define CONFIG_PCM_U24BE_DECODER 0
#define CONFIG_PCM_U24LE_DECODER 0
#define CONFIG_PCM_U32BE_DECODER 0
#define CONFIG_PCM_U32LE_DECODER 0
#define CONFIG_PCM_ZORK_DECODER 0
#define CONFIG_INTERPLAY_DPCM_DECODER 0
#define CONFIG_ROQ_DPCM_DECODER 0
#define CONFIG_SOL_DPCM_DECODER 0
#define CONFIG_XAN_DPCM_DECODER 0
#define CONFIG_ADPCM_4XM_DECODER 0
#define CONFIG_ADPCM_ADX_DECODER 0
#define CONFIG_ADPCM_CT_DECODER 0
#define CONFIG_ADPCM_EA_DECODER 0
#define CONFIG_ADPCM_EA_MAXIS_XA_DECODER 0
#define CONFIG_ADPCM_EA_R1_DECODER 0
#define CONFIG_ADPCM_EA_R2_DECODER 0
#define CONFIG_ADPCM_EA_R3_DECODER 0
#define CONFIG_ADPCM_EA_XAS_DECODER 0
#define CONFIG_ADPCM_G722_DECODER 0
#define CONFIG_ADPCM_G726_DECODER 0
#define CONFIG_ADPCM_IMA_AMV_DECODER 0
#define CONFIG_ADPCM_IMA_APC_DECODER 0
#define CONFIG_ADPCM_IMA_DK3_DECODER 0
#define CONFIG_ADPCM_IMA_DK4_DECODER 0
#define CONFIG_ADPCM_IMA_EA_EACS_DECODER 0
#define CONFIG_ADPCM_IMA_EA_SEAD_DECODER 0
#define CONFIG_ADPCM_IMA_ISS_DECODER 0
#define CONFIG_ADPCM_IMA_QT_DECODER 0
#define CONFIG_ADPCM_IMA_SMJPEG_DECODER 0
#define CONFIG_ADPCM_IMA_WAV_DECODER 0
#define CONFIG_ADPCM_IMA_WS_DECODER 0
#define CONFIG_ADPCM_MS_DECODER 0
#define CONFIG_ADPCM_SBPRO_2_DECODER 0
#define CONFIG_ADPCM_SBPRO_3_DECODER 0
#define CONFIG_ADPCM_SBPRO_4_DECODER 0
#define CONFIG_ADPCM_SWF_DECODER 0
#define CONFIG_ADPCM_THP_DECODER 0
#define CONFIG_ADPCM_XA_DECODER 0
#define CONFIG_ADPCM_YAMAHA_DECODER 0
#define CONFIG_ASS_DECODER 0
#define CONFIG_DVBSUB_DECODER 0
#define CONFIG_DVDSUB_DECODER 0
#define CONFIG_PGSSUB_DECODER 0
#define CONFIG_SRT_DECODER 0
#define CONFIG_XSUB_DECODER 0
#define CONFIG_LIBFDK_AAC_DECODER 0
#define CONFIG_LIBGSM_DECODER 0
#define CONFIG_LIBGSM_MS_DECODER 0
#define CONFIG_LIBILBC_DECODER 0
#define CONFIG_LIBOPENCORE_AMRNB_DECODER 0
#define CONFIG_LIBOPENCORE_AMRWB_DECODER 0
#define CONFIG_LIBOPENJPEG_DECODER 0
#define CONFIG_LIBOPUS_DECODER 0
#define CONFIG_LIBSCHROEDINGER_DECODER 0
#define CONFIG_LIBSPEEX_DECODER 0
#define CONFIG_LIBVPX_VP8_DECODER 0
#define CONFIG_LIBVPX_VP9_DECODER 0

#define CONFIG_MOV_DEMUXER 1
#define CONFIG_AAC_DEMUXER 0
#define CONFIG_AC3_DEMUXER 0
#define CONFIG_ADX_DEMUXER 0
#define CONFIG_AEA_DEMUXER 0
#define CONFIG_AIFF_DEMUXER 0
#define CONFIG_AMR_DEMUXER 0
#define CONFIG_ANM_DEMUXER 0
#define CONFIG_APC_DEMUXER 0
#define CONFIG_APE_DEMUXER 0
#define CONFIG_ASF_DEMUXER 0
#define CONFIG_ASS_DEMUXER 0
#define CONFIG_AU_DEMUXER 0
#define CONFIG_AVI_DEMUXER 0
#define CONFIG_AVISYNTH_DEMUXER 0
#define CONFIG_AVS_DEMUXER 0
#define CONFIG_BETHSOFTVID_DEMUXER 0
#define CONFIG_BFI_DEMUXER 0
#define CONFIG_BINK_DEMUXER 0
#define CONFIG_BMV_DEMUXER 0
#define CONFIG_C93_DEMUXER 0
#define CONFIG_CAF_DEMUXER 0
#define CONFIG_CAVSVIDEO_DEMUXER 0
#define CONFIG_CDG_DEMUXER 0
#define CONFIG_CDXL_DEMUXER 0
#define CONFIG_DAUD_DEMUXER 0
#define CONFIG_DFA_DEMUXER 0
#define CONFIG_DIRAC_DEMUXER 0
#define CONFIG_DNXHD_DEMUXER 0
#define CONFIG_DSICIN_DEMUXER 0
#define CONFIG_DTS_DEMUXER 0
#define CONFIG_DV_DEMUXER 0
#define CONFIG_DXA_DEMUXER 0
#define CONFIG_EA_DEMUXER 0
#define CONFIG_EA_CDATA_DEMUXER 0
#define CONFIG_EAC3_DEMUXER 0
#define CONFIG_FFM_DEMUXER 0
#define CONFIG_FFMETADATA_DEMUXER 0
#define CONFIG_FILMSTRIP_DEMUXER 0
#define CONFIG_FLAC_DEMUXER 0
#define CONFIG_FLIC_DEMUXER 0
#define CONFIG_FLV_DEMUXER 0
#define CONFIG_FOURXM_DEMUXER 0
#define CONFIG_G722_DEMUXER 0
#define CONFIG_G723_1_DEMUXER 0
#define CONFIG_GSM_DEMUXER 0
#define CONFIG_GXF_DEMUXER 0
#define CONFIG_H261_DEMUXER 0
#define CONFIG_H263_DEMUXER 0
#define CONFIG_H264_DEMUXER 0
#define CONFIG_HEVC_DEMUXER 0
#define CONFIG_HLS_DEMUXER 0
#define CONFIG_HNM_DEMUXER 0
#define CONFIG_IDCIN_DEMUXER 0
#define CONFIG_IFF_DEMUXER 0
#define CONFIG_ILBC_DEMUXER 0
#define CONFIG_IMAGE2_DEMUXER 0
#define CONFIG_IMAGE2PIPE_DEMUXER 0
#define CONFIG_INGENIENT_DEMUXER 0
#define CONFIG_IPMOVIE_DEMUXER 0
#define CONFIG_ISS_DEMUXER 0
#define CONFIG_IV8_DEMUXER 0
#define CONFIG_IVF_DEMUXER 0
#define CONFIG_JV_DEMUXER 0
#define CONFIG_LATM_DEMUXER 0
#define CONFIG_LMLM4_DEMUXER 0
#define CONFIG_LXF_DEMUXER 0
#define CONFIG_M4V_DEMUXER 0
#define CONFIG_MATROSKA_DEMUXER 0
#define CONFIG_MJPEG_DEMUXER 0
#define CONFIG_MLP_DEMUXER 0
#define CONFIG_MM_DEMUXER 0
#define CONFIG_MMF_DEMUXER 0
#define CONFIG_MP3_DEMUXER 0
#define CONFIG_MPC_DEMUXER 0
#define CONFIG_MPC8_DEMUXER 0
#define CONFIG_MPEGPS_DEMUXER 0
#define CONFIG_MPEGTS_DEMUXER 0
#define CONFIG_MPEGTSRAW_DEMUXER 0
#define CONFIG_MPEGVIDEO_DEMUXER 0
#define CONFIG_MSNWC_TCP_DEMUXER 0
#define CONFIG_MTV_DEMUXER 0
#define CONFIG_MVI_DEMUXER 0
#define CONFIG_MXF_DEMUXER 0
#define CONFIG_MXG_DEMUXER 0
#define CONFIG_NC_DEMUXER 0
#define CONFIG_NSV_DEMUXER 0
#define CONFIG_NUT_DEMUXER 0
#define CONFIG_NUV_DEMUXER 0
#define CONFIG_OGG_DEMUXER 0
#define CONFIG_OMA_DEMUXER 0
#define CONFIG_PCM_ALAW_DEMUXER 0
#define CONFIG_PCM_MULAW_DEMUXER 0
#define CONFIG_PCM_F64BE_DEMUXER 0
#define CONFIG_PCM_F64LE_DEMUXER 0
#define CONFIG_PCM_F32BE_DEMUXER 0
#define CONFIG_PCM_F32LE_DEMUXER 0
#define CONFIG_PCM_S32BE_DEMUXER 0
#define CONFIG_PCM_S32LE_DEMUXER 0
#define CONFIG_PCM_S24BE_DEMUXER 0
#define CONFIG_PCM_S24LE_DEMUXER 0
#define CONFIG_PCM_S16BE_DEMUXER 0
#define CONFIG_PCM_S16LE_DEMUXER 0
#define CONFIG_PCM_S8_DEMUXER 0
#define CONFIG_PCM_U32BE_DEMUXER 0
#define CONFIG_PCM_U32LE_DEMUXER 0
#define CONFIG_PCM_U24BE_DEMUXER 0
#define CONFIG_PCM_U24LE_DEMUXER 0
#define CONFIG_PCM_U16BE_DEMUXER 0
#define CONFIG_PCM_U16LE_DEMUXER 0
#define CONFIG_PCM_U8_DEMUXER 0
#define CONFIG_PMP_DEMUXER 0
#define CONFIG_PVA_DEMUXER 0
#define CONFIG_QCP_DEMUXER 0
#define CONFIG_R3D_DEMUXER 0
#define CONFIG_RAWVIDEO_DEMUXER 0
#define CONFIG_RL2_DEMUXER 0
#define CONFIG_RM_DEMUXER 0
#define CONFIG_ROQ_DEMUXER 0
#define CONFIG_RPL_DEMUXER 0
#define CONFIG_RSO_DEMUXER 0
#define CONFIG_RTP_DEMUXER 0
#define CONFIG_RTSP_DEMUXER 0
#define CONFIG_SAP_DEMUXER 0
#define CONFIG_SDP_DEMUXER 0
#define CONFIG_SEGAFILM_DEMUXER 0
#define CONFIG_SHORTEN_DEMUXER 0
#define CONFIG_SIFF_DEMUXER 0
#define CONFIG_SMACKER_DEMUXER 0
#define CONFIG_SMJPEG_DEMUXER 0
#define CONFIG_SOL_DEMUXER 0
#define CONFIG_SOX_DEMUXER 0
#define CONFIG_SPDIF_DEMUXER 0
#define CONFIG_SRT_DEMUXER 0
#define CONFIG_STR_DEMUXER 0
#define CONFIG_SWF_DEMUXER 0
#define CONFIG_TAK_DEMUXER 0
#define CONFIG_THP_DEMUXER 0
#define CONFIG_TIERTEXSEQ_DEMUXER 0
#define CONFIG_TMV_DEMUXER 0
#define CONFIG_TRUEHD_DEMUXER 0
#define CONFIG_TTA_DEMUXER 0
#define CONFIG_TXD_DEMUXER 0
#define CONFIG_TTY_DEMUXER 0
#define CONFIG_VC1_DEMUXER 0
#define CONFIG_VC1T_DEMUXER 0
#define CONFIG_VMD_DEMUXER 0
#define CONFIG_VOC_DEMUXER 0
#define CONFIG_VQF_DEMUXER 0
#define CONFIG_W64_DEMUXER 0
#define CONFIG_WAV_DEMUXER 0
#define CONFIG_WC3_DEMUXER 0
#define CONFIG_WSAUD_DEMUXER 0
#define CONFIG_WSVQA_DEMUXER 0
#define CONFIG_WTV_DEMUXER 0
#define CONFIG_WV_DEMUXER 0
#define CONFIG_XA_DEMUXER 0
#define CONFIG_XMV_DEMUXER 0
#define CONFIG_XWMA_DEMUXER 0
#define CONFIG_YOP_DEMUXER 0
#define CONFIG_YUV4MPEGPIPE_DEMUXER 0

#define CONFIG_FLV_ENCODER 1
#define CONFIG_H261_ENCODER 1
#define CONFIG_H263_ENCODER 1
#define CONFIG_MPEG4_ENCODER 1
#define CONFIG_WMV2_ENCODER 1
#define CONFIG_MPEG1VIDEO_ENCODER 1
#define CONFIG_MPEG2VIDEO_ENCODER 1
#define CONFIG_MSMPEG4V2_ENCODER 1
#define CONFIG_MSMPEG4V3_ENCODER 1
#define CONFIG_MJPEG_ENCODER 1
#define CONFIG_RV10_ENCODER 1
#define CONFIG_RV20_ENCODER 1
#define CONFIG_JPEGLS_ENCODER 1
#define CONFIG_AAC_ENCODER 1
#define CONFIG_A64MULTI_ENCODER 0
#define CONFIG_A64MULTI5_ENCODER 0
#define CONFIG_ASV1_ENCODER 0
#define CONFIG_ASV2_ENCODER 0
#define CONFIG_BMP_ENCODER 0
#define CONFIG_CLJR_ENCODER 0
#define CONFIG_COMFORTNOISE_ENCODER 0
#define CONFIG_DNXHD_ENCODER 0
#define CONFIG_DPX_ENCODER 0
#define CONFIG_DVVIDEO_ENCODER 0
#define CONFIG_FFV1_ENCODER 0
#define CONFIG_FFVHUFF_ENCODER 0
#define CONFIG_FLASHSV_ENCODER 0
#define CONFIG_FLV_ENCODER 1
#define CONFIG_GIF_ENCODER 0
#define CONFIG_H261_ENCODER 1
#define CONFIG_H263_ENCODER 1
#define CONFIG_H263P_ENCODER 0
#define CONFIG_HUFFYUV_ENCODER 0
#define CONFIG_JPEGLS_ENCODER 1
#define CONFIG_LJPEG_ENCODER 0
#define CONFIG_MJPEG_ENCODER 1
#define CONFIG_MPEG1VIDEO_ENCODER 1
#define CONFIG_MPEG2VIDEO_ENCODER 1
#define CONFIG_MPEG4_ENCODER 1
#define CONFIG_MSMPEG4V2_ENCODER 1
#define CONFIG_MSMPEG4V3_ENCODER 1
#define CONFIG_PAM_ENCODER 0
#define CONFIG_PBM_ENCODER 0
#define CONFIG_PCX_ENCODER 0
#define CONFIG_PGM_ENCODER 0
#define CONFIG_PGMYUV_ENCODER 0
#define CONFIG_PNG_ENCODER 0
#define CONFIG_PPM_ENCODER 0
#define CONFIG_PRORES_ENCODER 0
#define CONFIG_QTRLE_ENCODER 0
#define CONFIG_RAWVIDEO_ENCODER 0
#define CONFIG_ROQ_ENCODER 0
#define CONFIG_RV10_ENCODER 1
#define CONFIG_RV20_ENCODER 1
#define CONFIG_SGI_ENCODER 0
#define CONFIG_SUNRAST_ENCODER 0
#define CONFIG_SVQ1_ENCODER 0
#define CONFIG_TARGA_ENCODER 0
#define CONFIG_TIFF_ENCODER 0
#define CONFIG_UTVIDEO_ENCODER 0
#define CONFIG_V210_ENCODER 0
#define CONFIG_V410_ENCODER 0
#define CONFIG_WMV1_ENCODER 0
#define CONFIG_WMV2_ENCODER 1
#define CONFIG_XBM_ENCODER 0
#define CONFIG_XWD_ENCODER 0
#define CONFIG_ZLIB_ENCODER 0
#define CONFIG_ZMBV_ENCODER 0
#define CONFIG_AAC_ENCODER 1
#define CONFIG_AC3_ENCODER 0
#define CONFIG_AC3_FIXED_ENCODER 0
#define CONFIG_ALAC_ENCODER 0
#define CONFIG_EAC3_ENCODER 0
#define CONFIG_FLAC_ENCODER 0
#define CONFIG_MP2_ENCODER 0
#define CONFIG_NELLYMOSER_ENCODER 0
#define CONFIG_RA_144_ENCODER 0
#define CONFIG_VORBIS_ENCODER 0
#define CONFIG_WMAV1_ENCODER 0
#define CONFIG_WMAV2_ENCODER 0
#define CONFIG_PCM_ALAW_ENCODER 0
#define CONFIG_PCM_F32BE_ENCODER 0
#define CONFIG_PCM_F32LE_ENCODER 0
#define CONFIG_PCM_F64BE_ENCODER 0
#define CONFIG_PCM_F64LE_ENCODER 0
#define CONFIG_PCM_MULAW_ENCODER 0
#define CONFIG_PCM_S8_ENCODER 0
#define CONFIG_PCM_S16BE_ENCODER 0
#define CONFIG_PCM_S16LE_ENCODER 0
#define CONFIG_PCM_S24BE_ENCODER 0
#define CONFIG_PCM_S24DAUD_ENCODER 0
#define CONFIG_PCM_S24LE_ENCODER 0
#define CONFIG_PCM_S32BE_ENCODER 0
#define CONFIG_PCM_S32LE_ENCODER 0
#define CONFIG_PCM_U8_ENCODER 0
#define CONFIG_PCM_U16BE_ENCODER 0
#define CONFIG_PCM_U16LE_ENCODER 0
#define CONFIG_PCM_U24BE_ENCODER 0
#define CONFIG_PCM_U24LE_ENCODER 0
#define CONFIG_PCM_U32BE_ENCODER 0
#define CONFIG_PCM_U32LE_ENCODER 0
#define CONFIG_ROQ_DPCM_ENCODER 0
#define CONFIG_ADPCM_ADX_ENCODER 0
#define CONFIG_ADPCM_G722_ENCODER 0
#define CONFIG_ADPCM_G726_ENCODER 0
#define CONFIG_ADPCM_IMA_QT_ENCODER 0
#define CONFIG_ADPCM_IMA_WAV_ENCODER 0
#define CONFIG_ADPCM_MS_ENCODER 0
#define CONFIG_ADPCM_SWF_ENCODER 0
#define CONFIG_ADPCM_YAMAHA_ENCODER 0
#define CONFIG_ASS_ENCODER 0
#define CONFIG_DVBSUB_ENCODER 0
#define CONFIG_DVDSUB_ENCODER 0
#define CONFIG_XSUB_ENCODER 0
#define CONFIG_LIBFAAC_ENCODER 0
#define CONFIG_LIBFDK_AAC_ENCODER 0
#define CONFIG_LIBGSM_ENCODER 0
#define CONFIG_LIBGSM_MS_ENCODER 0
#define CONFIG_LIBILBC_ENCODER 0
#define CONFIG_LIBMP3LAME_ENCODER 0
#define CONFIG_LIBOPENCORE_AMRNB_ENCODER 0
#define CONFIG_LIBOPENJPEG_ENCODER 0
#define CONFIG_LIBOPUS_ENCODER 0
#define CONFIG_LIBSCHROEDINGER_ENCODER 0
#define CONFIG_LIBSPEEX_ENCODER 0
#define CONFIG_LIBTHEORA_ENCODER 0
#define CONFIG_LIBVO_AACENC_ENCODER 0
#define CONFIG_LIBVO_AMRWBENC_ENCODER 0
#define CONFIG_LIBVORBIS_ENCODER 0
#define CONFIG_LIBVPX_VP8_ENCODER 0
#define CONFIG_LIBVPX_VP9_ENCODER 0
#define CONFIG_LIBWAVPACK_ENCODER 0
#define CONFIG_LIBWEBP_ENCODER 0
#define CONFIG_LIBX264_ENCODER 0
#define CONFIG_LIBXAVS_ENCODER 0
#define CONFIG_LIBXVID_ENCODER 0

#define CONFIG_MOV_MUXER 1
#define CONFIG_MP4_MUXER 1
#define CONFIG_A64_MUXER 0
#define CONFIG_AC3_MUXER 0
#define CONFIG_ADTS_MUXER 0
#define CONFIG_ADX_MUXER 0
#define CONFIG_AIFF_MUXER 0
#define CONFIG_AMR_MUXER 0
#define CONFIG_ASF_MUXER 0
#define CONFIG_ASS_MUXER 0
#define CONFIG_ASF_STREAM_MUXER 0
#define CONFIG_AU_MUXER 0
#define CONFIG_AVI_MUXER 0
#define CONFIG_AVM2_MUXER 0
#define CONFIG_CAVSVIDEO_MUXER 0
#define CONFIG_CRC_MUXER 0
#define CONFIG_DAUD_MUXER 0
#define CONFIG_DIRAC_MUXER 0
#define CONFIG_DNXHD_MUXER 0
#define CONFIG_DTS_MUXER 0
#define CONFIG_DV_MUXER 0
#define CONFIG_EAC3_MUXER 0
#define CONFIG_F4V_MUXER 0
#define CONFIG_FFM_MUXER 0
#define CONFIG_FFMETADATA_MUXER 0
#define CONFIG_FILMSTRIP_MUXER 0
#define CONFIG_FLAC_MUXER 0
#define CONFIG_FLV_MUXER 0
#define CONFIG_FRAMECRC_MUXER 0
#define CONFIG_FRAMEMD5_MUXER 0
#define CONFIG_G722_MUXER 0
#define CONFIG_GIF_MUXER 0
#define CONFIG_GXF_MUXER 0
#define CONFIG_H261_MUXER 0
#define CONFIG_H263_MUXER 0
#define CONFIG_H264_MUXER 0
#define CONFIG_HDS_MUXER 0
#define CONFIG_HEVC_MUXER 0
#define CONFIG_HLS_MUXER 0
#define CONFIG_ILBC_MUXER 0
#define CONFIG_IMAGE2_MUXER 0
#define CONFIG_IMAGE2PIPE_MUXER 0
#define CONFIG_IPOD_MUXER 0
#define CONFIG_ISMV_MUXER 0
#define CONFIG_IVF_MUXER 0
#define CONFIG_LATM_MUXER 0
#define CONFIG_M4V_MUXER 0
#define CONFIG_MD5_MUXER 0
#define CONFIG_MATROSKA_MUXER 0
#define CONFIG_MATROSKA_AUDIO_MUXER 0
#define CONFIG_MJPEG_MUXER 0
#define CONFIG_MLP_MUXER 0
#define CONFIG_MMF_MUXER 0
#define CONFIG_MP2_MUXER 0
#define CONFIG_MP3_MUXER 0
#define CONFIG_MPEG1SYSTEM_MUXER 0
#define CONFIG_MPEG1VCD_MUXER 0
#define CONFIG_MPEG1VIDEO_MUXER 0
#define CONFIG_MPEG2DVD_MUXER 0
#define CONFIG_MPEG2SVCD_MUXER 0
#define CONFIG_MPEG2VIDEO_MUXER 0
#define CONFIG_MPEG2VOB_MUXER 0
#define CONFIG_MPEGTS_MUXER 0
#define CONFIG_MPJPEG_MUXER 0
#define CONFIG_MXF_MUXER 0
#define CONFIG_MXF_D10_MUXER 0
#define CONFIG_NULL_MUXER 0
#define CONFIG_NUT_MUXER 0
#define CONFIG_OGG_MUXER 0
#define CONFIG_OMA_MUXER 0
#define CONFIG_PCM_ALAW_MUXER 0
#define CONFIG_PCM_MULAW_MUXER 0
#define CONFIG_PCM_F64BE_MUXER 0
#define CONFIG_PCM_F64LE_MUXER 0
#define CONFIG_PCM_F32BE_MUXER 0
#define CONFIG_PCM_F32LE_MUXER 0
#define CONFIG_PCM_S32BE_MUXER 0
#define CONFIG_PCM_S32LE_MUXER 0
#define CONFIG_PCM_S24BE_MUXER 0
#define CONFIG_PCM_S24LE_MUXER 0
#define CONFIG_PCM_S16BE_MUXER 0
#define CONFIG_PCM_S16LE_MUXER 0
#define CONFIG_PCM_S8_MUXER 0
#define CONFIG_PCM_U32BE_MUXER 0
#define CONFIG_PCM_U32LE_MUXER 0
#define CONFIG_PCM_U24BE_MUXER 0
#define CONFIG_PCM_U24LE_MUXER 0
#define CONFIG_PCM_U16BE_MUXER 0
#define CONFIG_PCM_U16LE_MUXER 0
#define CONFIG_PCM_U8_MUXER 0
#define CONFIG_PSP_MUXER 0
#define CONFIG_RAWVIDEO_MUXER 0
#define CONFIG_RM_MUXER 0
#define CONFIG_ROQ_MUXER 0
#define CONFIG_RSO_MUXER 0
#define CONFIG_RTP_MUXER 0
#define CONFIG_RTSP_MUXER 0
#define CONFIG_SAP_MUXER 0
#define CONFIG_SEGMENT_MUXER 0
#define CONFIG_SMJPEG_MUXER 0
#define CONFIG_SMOOTHSTREAMING_MUXER 0
#define CONFIG_SOX_MUXER 0
#define CONFIG_SPDIF_MUXER 0
#define CONFIG_SRT_MUXER 0
#define CONFIG_SWF_MUXER 0
#define CONFIG_TG2_MUXER 0
#define CONFIG_TGP_MUXER 0
#define CONFIG_TRUEHD_MUXER 0
#define CONFIG_VC1T_MUXER 0
#define CONFIG_VOC_MUXER 0
#define CONFIG_WAV_MUXER 0
#define CONFIG_WEBM_MUXER 0
#define CONFIG_WV_MUXER 0
#define CONFIG_YUV4MPEGPIPE_MUXER 0
