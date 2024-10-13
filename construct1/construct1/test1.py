#!/usr/bin/env python
import rclpy

def main(args=None):
    rclpy.init(args=args)
    print('hello')
    rclpy.shutdown()

if __name__=='__main__':
    main()