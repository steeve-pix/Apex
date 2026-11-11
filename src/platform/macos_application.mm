#import <AppKit/AppKit.h>

#include "platform/macos_application.hpp"

#include <stdexcept>

namespace apex::platform {
    void set_application_icon(const std::string& path) {
        @autoreleasepool {
            NSString* filename =
                [NSString stringWithUTF8String:path.c_str()];

            NSImage* image =
                [[NSImage alloc] initWithContentsOfFile:filename];

            if (!image) {
                throw std::runtime_error(
                    "Failed to load application icon"
                );
            }

            [NSApp setApplicationIconImage:image];
            [image release];
        }
    }
}