package frc.robot.utils;

import java.io.IOException;
import java.io.PrintStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;

/**
 * FRC机器人消息发送类
 * 提供8个不同类别的日志输出方法
 */
public class MessageSender {
    
    private static DatagramSocket socket;
    private static InetAddress broadcastAddress;
    private static final int DEFAULT_UDP_PORT = 7777; // 端口号提取成常数
    private static int udpPort = DEFAULT_UDP_PORT;
    private static boolean initialized = false;
    private static PrintStream originalErr;
    
    /**
     * 初始化消息发送系统
     * @param port UDP端口号
     */
    public static void init(int port) {
        if (initialized) {
            return;
        }
        
        try {
            udpPort = port;
            socket = new DatagramSocket();
            socket.setBroadcast(true); // 启用广播
            broadcastAddress = InetAddress.getByName("255.255.255.255"); // 广播地址
            originalErr = System.err; // 保存原始的错误流
            
            // 重定向System.err来捕获所有异常
            System.setErr(new PrintStream(new UDPOutputStream(originalErr)));
            
            initialized = true;

            log0("---------------------------- 重新开始 这是分割线 ----------------------------");
            log0("---------------------------- 重新开始 这是分割线 ----------------------------");
            log0("---------------------------- 重新开始 这是分割线 ----------------------------");
        } catch (SocketException | UnknownHostException e) {
            System.err.println("Failed to initialize MessageSender: " + e.getMessage());
        }
    }
    
    /**
     * 初始化消息发送系统，使用默认端口
     */
    public static void init() {
        init(DEFAULT_UDP_PORT);
    }
    
    /**
     * 内部方法：发送UDP消息
     */
    private static void sendUdpMessage(String message) {
        if (!initialized) {
            System.err.println("MessageSender not initialized. Call init() first.");
            return;
        }
        
        try {
            byte[] buffer = message.getBytes("UTF-8");
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length, broadcastAddress, udpPort);
            socket.send(packet);
            System.out.println(message);
        } catch (IOException e) {
            originalErr.println("Failed to send UDP message: " + e.getMessage());
        }
    }
    
    /**
     * 发送默认类别消息 (ID: 0)
     * @param message 要发送的消息内容
     */
    public static void log0(String message) {
        sendUdpMessage("<--0-->" + message);
    }
    
    /**
     * 发送信息类别消息 (ID: 1)
     * @param message 要发送的消息内容
     */
    public static void log1(String message) {
        sendUdpMessage("<--1-->" + message);
    }
    
    /**
     * 发送警告类别消息 (ID: 2)
     * @param message 要发送的消息内容
     */
    public static void log2(String message) {
        sendUdpMessage("<--2-->" + message);
    }
    
    /**
     * 发送错误类别消息 (ID: 3)
     * @param message 要发送的消息内容
     */
    public static void log3(String message) {
        sendUdpMessage("<--3-->" + message);
    }
    
    /**
     * 发送调试类别消息 (ID: 4)
     * @param message 要发送的消息内容
     */
    public static void log4(String message) {
        sendUdpMessage("<--4-->" + message);
    }
    
    /**
     * 发送特殊类别消息 (ID: 5)
     * @param message 要发送的消息内容
     */
    public static void log5(String message) {
        sendUdpMessage("<--5-->" + message);
    }
    
    /**
     * 发送事件类别消息 (ID: 6)
     * @param message 要发送的消息内容
     */
    public static void log6(String message) {
        sendUdpMessage("<--6-->" + message);
    }
    
    /**
     * 发送状态类别消息 (ID: 7)
     * @param message 要发送的消息内容
     */
    public static void log7(String message) {
        sendUdpMessage("<--7-->" + message);
    }

    public static void logException(String message) {
        log3(message);
    }
    public static void logError(String message) {
        log3(message);
    }

    public static void log(String message) {
        log1(message);
    }

    public static void logWarning(String message) {
        log2(message);
    }
    
    // /**
    //  * 通用日志方法，允许指定类别ID
    //  * @param categoryId 类别ID (0-7)
    //  * @param message 要发送的消息内容
    //  */
    // public static void log(int categoryId, String message) {
    //     if (categoryId >= 0 && categoryId <= 7) {
    //         sendUdpMessage("<--" + categoryId + "-->" + message);
    //     } else {
    //         // 如果类别ID超出范围，默认使用类别0
    //         sendUdpMessage("<--0-->" + "[INVALID CATEGORY ID: " + categoryId + "] " + message);
    //     }
    // }
    
    /**
     * 用于捕获系统异常的输出流
     */
    private static class UDPOutputStream extends java.io.OutputStream {
        private final PrintStream originalStream;
        private final StringBuilder buffer = new StringBuilder();
        
        public UDPOutputStream(PrintStream originalStream) {
            this.originalStream = originalStream;
        }
        
        @Override
        public void write(int b) throws IOException {
            char c = (char) b;
            buffer.append(c);
            
            // 如果遇到换行符，发送整行
            if (c == '\n') {
                String line = buffer.toString();
                
                // 原始输出
                originalStream.print(line);
                
                // 发送到UDP
                logException(line.trim());
                buffer.setLength(0); // 清空缓冲区
            }
        }
        
        @Override
        public void write(byte[] b, int off, int len) throws IOException {
            String str = new String(b, off, len, "UTF-8");
            
            // 按行处理字符串
            String[] lines = str.split("\n", -1);
            for (int i = 0; i < lines.length; i++) {
                buffer.append(lines[i]);
                
                // 如果不是最后一行（意味着原字符串以\n结尾），则发送整行
                if (i < lines.length - 1 || str.endsWith("\n")) {
                    String line = buffer.toString();
                    
                    // 原始输出
                    originalStream.print(line + (i < lines.length - 1 ? "\n" : ""));
                    
                    // 发送到UDP
                    logException(line.trim());
                    
                    buffer.setLength(0); // 清空缓冲区
                }
            }
        }
    }
}