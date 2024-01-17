#![allow(non_camel_case_types)]
#![allow(dead_code)]
#![allow(unused)]

/**
 * @brief - implements Raw sockets in Rust.
 *
 * This implementation of raw socket uses the underlying rust libc wrapper.
 *
 * include libc in dependencies section in Cargo.toml.
 *
 * @copyright - 2024-present. Devendra Naga. All rights reserved.
 * See LICENSE.
 */
use libc::{SOCK_RAW, ifreq, SIOCGIFFLAGS, IFF_PROMISC, SIOCSIFFLAGS, SIOCGIFHWADDR, SIOCGIFINDEX, SOL_SOCKET, SO_BINDTODEVICE, sockaddr_alg};
use core::ffi::c_void;
use core::mem;

/**
 * @brief - Defines a basic Raw_Socket context structure.
 */
struct Raw_Socket {
    //
    // hides the fd and mac from user
    fd : i32,
    mac : [u8; 6],
    //
    // to fill into the send function later
    ifindex : i32,
}

/**
 * @brief - i implemented this trait to cover for initialization of
 * sockaddr_ll and nothing else.
 */
trait sockaddr_ll_trait {
    /**
     * @brief - init the sockaddr_ll and return it.
     *
     * @return - returns libc::sockaddr_ll.
     */
    fn new() -> libc::sockaddr_ll {
        libc::sockaddr_ll {
            sll_family : libc::AF_PACKET as u16,
            sll_protocol : libc::ETH_P_ALL as u16,
            sll_ifindex : 0,
            sll_hatype : 0,
            sll_pkttype : 0,
            sll_halen : 0,
            sll_addr : Default::default()
        }
    }
}

/**
 * @brief - empty impl for trait usage
 */
impl sockaddr_ll_trait for libc::sockaddr_ll {
}

/**
 * @brief - i implemented this trait to cover for initialization of
 * ifreq and nothing else.
 */
trait ifreq_trait {
    /**
     * @brief - zeroize ifreq.
     *
     * @return - zeroized ifreq.
     */
    fn new() -> ifreq {
        ifreq {
            ifr_name : Default::default(),
            ifr_ifru : {
                //
                // we init here the largest possible struct in the
                // union __c_anonymous_ifr_ifru. This tries to avoid
                // any possible uninitialized portions of this struct.
                //
                libc::__c_anonymous_ifr_ifru {
                    ifru_map : {
                        libc::__c_anonymous_ifru_map {
                            mem_start : 0,
                            mem_end : 0,
                            base_addr : 0,
                            irq : 0,
                            dma : 0,
                            port : 0
                        }
                    }
                }
            }
        }
    }
}

/**
 * @brief - holder impl for trait usage
 */
impl ifreq_trait for ifreq {
}

/**
 * @brief - set the interface in promiscous mode.
 *
 * This helps in capturing packets at Raw socket level.
 *
 * This function can be moved to an impl and rewritten better.
 *
 * @param [in] fd - socket fd.
 * @param [in] dev - interface.
 *
 * @return 0 on success -1 on failure.
 */
pub fn set_promisc(fd: i32, dev : &String) -> i32 {
    let mut i : usize = 0;
    let mut req : libc::ifreq = <ifreq as ifreq_trait>::new();
    let mut ret : i32;

    //
    // no good. TBD XXX find a better way to copy strings from
    // rust String to ::* c_char
    for c in dev.chars() {
        req.ifr_name[i] = c as i8;
        i = i + 1;
    }

    req.ifr_name[i] = '\0' as i8;

    unsafe {
        ret = libc::ioctl(fd, SIOCGIFFLAGS, &req);
        if ret < 0 {
            return -1;
        }

        req.ifr_ifru.ifru_flags |= IFF_PROMISC as i16;

        ret = libc::ioctl(fd, SIOCSIFFLAGS, &req);
        if ret < 0 {
            return -1;
        }
    }

    return 0;
}

/**
 * @brief - get the mac address from an interface.
 *
 * @param [in] fd - socket fd.
 * @param [in] dev - interface name.
 *
 * @return macaddress with non zero value on success.
 * @return macaddress with all zeros on failure.
 */
pub fn get_mac(fd : i32, dev : &String) -> [u8; 6] {
    let mut i : usize = 0;
    let mut req : libc::ifreq = <ifreq as ifreq_trait>::new();
    let ret : i32;
    let mut mac : [u8; 6] = Default::default();

    //
    // no good. TBD XXX find a better way to copy strings from
    // rust String to ::* c_char
    for c in dev.chars() {
        req.ifr_name[i] = c as i8;
        i = i + 1;
    }

    req.ifr_name[i] = '\0' as i8;

    unsafe {
        ret = libc::ioctl(fd, SIOCGIFHWADDR, &req);
        if ret < 0 {
            return mac;
        }

        for i in 0..6 {
            mac[i] = req.ifr_ifru.ifru_hwaddr.sa_data[i] as u8;
        }
    }

    return mac;
}

/**
 * @brief - empty holder for libc_socket to group socket related functions.
 */
struct libc_socket {

}

impl libc_socket {
    /**
     * @brief - bind the socket to the interface. Without this the reception
     * wont' work well.
     *
     * There's a better way to write this function.
     *
     * We also return ifindex .. but this can be split into another function.
     * too lazy.
     *
     * @param [in] fd - socket descriptor.
     * @param [in] dev - interface name.
     *
     * @return interface index on success.
     * @return -1 on failure.
     */
    pub fn bind_to_device(fd : i32, dev: &String) -> i32 {
        let mut i : usize = 0;
        let mut req : libc::ifreq = <ifreq as ifreq_trait>::new();
        let mut ifindex : i32 = -1;
        let mut ret : i32;

        //
        // no good. TBD XXX find a better way to copy strings from
        // rust String to ::* c_char
        for c in dev.chars() {
            req.ifr_name[i] = c as i8;
            i = i + 1;
        }

        req.ifr_name[i] = '\0' as i8;

        ret = unsafe { libc::ioctl(fd, SIOCGIFINDEX, &req) };
        if ret < 0 {
            return -1;
        }

        ifindex = unsafe { req.ifr_ifru.ifru_ifindex };

        ret = unsafe { libc::setsockopt(fd,
                                   SOL_SOCKET,
                                   SO_BINDTODEVICE,
                                   &req as *const libc::ifreq as *const c_void,
                                   mem::size_of_val(&req) as u32) };
        if ret < 0 {
            return -1;
        }
        return ifindex;
    }
    /**
     * @brief - close the socket.
     *
     * @param [in] fd - socket descriptor
     */
    pub fn close(fd : i32) {
        if fd > 0 {
            unsafe { libc::close(fd); }
        }
    }
}

/**
 * @brief - implementation of Raw_Socket methods.
 */
impl Raw_Socket {
    /**
     * @brief - initialize the Raw_Socket structure before using.
     *
     * @return - initialized Raw_Socket struct.
     */
    fn new() -> Raw_Socket {
        Raw_Socket {
            fd : -1,
            mac : Default::default(),
            ifindex : -1
        }
    }

    /**
     * @brief - bind the socket but only supports sockaddr_ll.
     *
     * @param [in] fd - socket descriptor.
     * @param [in] ifindex - interface index.
     *
     * @return 0 on success -1 on failure.
     */
    fn socket_bind(fd : i32, ifindex : i32) -> i32 {
        let mut lladdr : libc::sockaddr_ll = <libc::sockaddr_ll as sockaddr_ll_trait>::new();
        let mut ret : i32;

        lladdr.sll_ifindex = ifindex;
        lladdr.sll_protocol = (libc::ETH_P_ALL as u16).to_be();
        lladdr.sll_family = libc::AF_PACKET as u16;

        unsafe {
            ret = libc::bind(fd,
                            &lladdr as *const libc::sockaddr_ll as *const libc::sockaddr,
                            mem::size_of_val(&lladdr) as u32);
            if ret < 0 {
                return -1;
            }
        }

        return 0;
    }

    /**
     * @brief - initializes the raw socket and returns Raw_Socket.
     *
     * use this returned portion for rest of the operations.
     *
     * on failure, the rs.fd is always -1.
     *
     * @param [in] dev - interface name.
     *
     * @return Raw_Socket with all filled information on success.
     * @return Raw_Socket::fd = -1 on failure.
     */
    pub fn init(dev : &String) -> Raw_Socket {
        let mut req : libc::ifreq = <ifreq as ifreq_trait>::new();
        let mut rs : Raw_Socket = Raw_Socket::new();
        let mut ret : i32;

        unsafe {
            rs.fd = libc::socket(libc::AF_PACKET,
                                 libc::SOCK_RAW,
                                 (libc::ETH_P_ALL as u16).to_be() as i32);
            if rs.fd < 0 {
                return rs;
            }

            //
            // set promiscous mode
            ret = set_promisc(rs.fd, dev);
            if ret < 0 {
                libc_socket::close(rs.fd);
                rs.fd = -1;
                return rs;
            }

            //
            // get the mac address and keep it in context
            rs.mac = get_mac(rs.fd, dev);
            rs.ifindex = libc_socket::bind_to_device(rs.fd, dev);
            if rs.ifindex < 0 {
                libc_socket::close(rs.fd);
                rs.fd = -1;
                return rs;
            }

            //
            // bind the socket to the device
            ret = Raw_Socket::socket_bind(rs.fd, rs.ifindex);
            if ret < 0 {
                libc_socket::close(rs.fd);
                rs.fd = -1;
                return rs;
            }
        }

        return rs;
    }

    /**
     * @brief - send a packet over raw socket.
     *
     * @param [in] data - array of u8s
     * @param [in] data_len - length of data.
     *
     * @return non zero value on success.
     * @return 0 or -1 on failure.
     */
    pub fn send(&self, dst_mac : &[u8], data : &[u8], data_len : usize) -> i32 {
        let mut lladdr : libc::sockaddr_ll = <libc::sockaddr_ll as sockaddr_ll_trait>::new();
        let mut ret : isize;

        lladdr.sll_ifindex = self.ifindex;
        lladdr.sll_halen = 6;

        lladdr.sll_addr[0] = dst_mac[0];
        lladdr.sll_addr[1] = dst_mac[1];
        lladdr.sll_addr[2] = dst_mac[2];
        lladdr.sll_addr[3] = dst_mac[3];
        lladdr.sll_addr[4] = dst_mac[4];
        lladdr.sll_addr[5] = dst_mac[5];

        unsafe {
            ret = libc::sendto(self.fd,
                            data as *const [u8] as *const c_void,
                            data_len,
                            0,
                            &lladdr as *const libc::sockaddr_ll as *const libc::sockaddr,
                            mem::size_of_val(&lladdr) as u32);
            if ret < 0 {
                return ret as i32;
            }
        }

        return ret as i32;
    }

    /**
     * @brief - receive a packet over raw socket.
     *
     * @param [in] dst_mac - destination mac address.
     * @param [in] data - array of u8s.
     * @param [in] data_len - length of data.
     *
     * @return non zero value on success.
     * @return 0 or -1 on failure.
     */
    pub fn recv(&self, dst_mac : &[u8], data : &mut [u8], data_len : usize) -> i32 {
        let mut lladdr : libc::sockaddr_ll = <libc::sockaddr_ll as sockaddr_ll_trait>::new();
        let mut len : libc::socklen_t = mem::size_of_val(&lladdr) as u32;
        let mut ret : isize;

        unsafe {
            ret = libc::recvfrom(self.fd,
                                 data as *mut [u8] as *mut c_void,
                                 data_len,
                                 0,
                                 &mut lladdr as *mut libc::sockaddr_ll as *mut libc::sockaddr,
                                 &mut len as *mut u32);
            if ret < 0 {
                return ret as i32;
            }
        }

        return ret as i32;
    }
}

/**
 * @brief - defines a test code for raw socket.
 */
fn main() {
    let mut rs : Raw_Socket = Raw_Socket::init(&"wlp4s0".to_string());
    let mut ret : i32;

    if rs.fd < 0 {
        panic!("failed to init Raw Socket");
    }

    let mut dst_mac : [u8; 6] = Default::default();
    let mut data : [u8; 2048] = [0; 2048];
    let data_len : usize = data.len();

    loop {

        ret = rs.recv(&dst_mac, &mut data, data_len);
        println!("received packet with length { }", ret);
    }
}
