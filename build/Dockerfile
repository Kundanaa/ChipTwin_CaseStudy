# FROM --platform=linux/arm64 antmicro/renode:latest

# # Switch to root for package installation
# USER root

# # Install dependencies with proper permissions
# RUN apt-get update && \
#     apt-get install -y mono-complete && \
#     apt-get clean && \
#     rm -rf /var/lib/apt/lists/*

# # Switch back to default user
# USER renode
# # FROM --platform=linux/arm64 antmicro/renode:latest

# # # Create 'renode' user explicitly
# # RUN useradd -m renode && \
# #     apt-get update && \
# #     apt-get install -y mono-complete && \
# #     chown -R renode:renode /home/renode

# # USER renode
# WORKDIR /home/renode
FROM --platform=linux/arm64 antmicro/renode:latest

# Switch to root for package installation
USER root

# Update and install Mono
RUN apt-get update && \
    apt-get install -y mono-complete && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Restore 'renode' user and set proper permissions
USER renode
WORKDIR /home/renode

