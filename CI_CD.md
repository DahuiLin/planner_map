# CI/CD Documentation

## Overview

This repository includes comprehensive CI/CD workflows using GitHub Actions to ensure code quality and facilitate deployment of the Planner Map system.

## Workflows

### 1. CI - Build and Test (`ci.yml`)

**Triggers:**
- Push to `main`, `develop`, or `claude/**` branches
- Pull requests to `main` or `develop`
- Manual workflow dispatch

**Jobs:**

#### Docker Build and Test
- Builds both ROS2 and Web Docker images
- Starts services using Docker Compose
- Verifies container health
- Tests Web API endpoints
- Checks ROS2 nodes and topics
- Runs Python unit tests
- Runs integration tests
- Collects logs on failure

#### Lint and Code Quality
- Sets up Python environment
- Installs linting tools (flake8, pylint)
- Checks for syntax errors
- Identifies TODO/FIXME comments
- Finds debug print statements

#### Docker Compose Validation
- Validates docker compose.yml syntax
- Checks for required Docker files

### 2. Deploy (`deploy.yml`)

**Triggers:**
- Push to version tags (`v*.*.*`)
- Manual workflow dispatch with environment selection

**Jobs:**

#### Build and Push
- Builds Docker images with proper tags
- Pushes to Docker Hub (requires secrets)
- Uses layer caching for faster builds
- Tags images with version and SHA

#### Deploy
- Deploys to staging or production environment
- Runs deployment verification checks

#### Create Release
- Creates GitHub release for version tags
- Includes deployment instructions

## Required Secrets

To enable full CI/CD functionality, configure these secrets in your GitHub repository:

### Docker Hub (for deployment)
- `DOCKER_USERNAME`: Your Docker Hub username
- `DOCKER_PASSWORD`: Your Docker Hub password or access token

### Deployment (optional)
- Add SSH keys or cloud provider credentials as needed for your deployment target

## Health Checks

Both services include health checks in `docker compose.yml`:

### ROS2 Service
- **Test**: Checks if ROS2 nodes are running
- **Interval**: Every 30 seconds
- **Timeout**: 10 seconds
- **Retries**: 3
- **Start Period**: 40 seconds

### Web Service
- **Test**: HTTP request to `/api/status`
- **Interval**: Every 30 seconds
- **Timeout**: 10 seconds
- **Retries**: 3
- **Start Period**: 30 seconds

## Integration Tests

The `test_integration.py` script tests:
- Web API status endpoint
- Web interface HTML loading
- Map API endpoint
- Goal submission
- Path retrieval

### Running Tests Locally

```bash
# Start services
docker compose up -d

# Wait for services to be ready
sleep 30

# Install test dependencies
pip install requests

# Run integration tests
python3 test_integration.py
```

## CI Workflow Details

### Build Process

1. **Checkout**: Fetches the repository code
2. **Docker Buildx Setup**: Enables advanced Docker build features
3. **Cache**: Restores cached Docker layers for faster builds
4. **Build Images**: Builds ROS2 and Web images
5. **Start Services**: Launches containers with docker compose
6. **Health Checks**: Verifies services are running properly
7. **API Tests**: Tests Web API endpoints
8. **ROS2 Tests**: Checks ROS2 nodes and topics
9. **Integration Tests**: Runs end-to-end tests
10. **Cleanup**: Stops services and collects logs if needed

### Expected Build Time

- **First run**: ~15-20 minutes (no cache)
- **Subsequent runs**: ~5-10 minutes (with cache)
- **Total timeout**: 30 minutes

## Deployment Process

### Version Releases

1. **Tag a version**:
   ```bash
   git tag v1.0.0
   git push origin v1.0.0
   ```

2. **Automated process**:
   - GitHub Actions builds Docker images
   - Images are tagged with version
   - Images are pushed to Docker Hub
   - GitHub release is created automatically

3. **Deploy the release**:
   ```bash
   docker pull <your-username>/planner_map_ros2:v1.0.0
   docker pull <your-username>/planner_map_web:v1.0.0
   docker compose up -d
   ```

### Manual Deployment

1. Navigate to Actions tab in GitHub
2. Select "Deploy to Production" workflow
3. Click "Run workflow"
4. Choose environment (staging/production)
5. Monitor deployment progress

## Monitoring Builds

### View Workflow Status

- Visit the "Actions" tab in your GitHub repository
- Select a workflow run to see details
- Check individual job logs

### Build Badges

Add build status badges to your README:

```markdown
![CI](https://github.com/DahuiLin/planner_map/workflows/CI%20-%20Build%20and%20Test/badge.svg)
![Deploy](https://github.com/DahuiLin/planner_map/workflows/Deploy%20to%20Production/badge.svg)
```

## Troubleshooting

### Build Fails on Docker Build

**Issue**: Docker build timeout or resource limits

**Solutions**:
- Check Dockerfile syntax
- Ensure base images are available
- Review build logs for specific errors
- Increase timeout in workflow if needed

### Health Checks Failing

**Issue**: Containers start but health checks fail

**Solutions**:
- Check container logs: `docker compose logs`
- Verify health check commands work inside containers
- Increase `start_period` in health check configuration
- Ensure dependencies are properly configured

### Integration Tests Failing

**Issue**: API endpoints not responding

**Solutions**:
- Check if containers are running: `docker compose ps`
- Verify network connectivity between containers
- Check web service logs for errors
- Ensure ROS2 nodes are started properly

### Deployment Fails

**Issue**: Cannot push to Docker Hub

**Solutions**:
- Verify Docker Hub credentials in secrets
- Check secret names match workflow configuration
- Ensure Docker Hub repository exists
- Verify account permissions

## Best Practices

1. **Run tests locally** before pushing:
   ```bash
   make dev  # Build and run with logs
   python3 test_integration.py
   ```

2. **Keep workflows fast**:
   - Use Docker layer caching
   - Run jobs in parallel when possible
   - Set appropriate timeouts

3. **Monitor resource usage**:
   - GitHub Actions has usage limits
   - Private repositories: 2000 minutes/month on free tier
   - Public repositories: Unlimited

4. **Version tagging**:
   - Use semantic versioning (v1.0.0, v1.1.0, etc.)
   - Create tags from stable commits
   - Document changes in release notes

## Extending CI/CD

### Adding New Tests

1. Create test file in repository root
2. Update `.github/workflows/ci.yml` to run your tests
3. Ensure tests can run in CI environment

### Adding Deployment Targets

1. Edit `.github/workflows/deploy.yml`
2. Add deployment job for your target
3. Configure necessary secrets
4. Test deployment process

### Custom Workflows

Create new workflows in `.github/workflows/`:

```yaml
name: Custom Workflow
on:
  push:
    branches: [main]
jobs:
  custom-job:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Custom step
        run: echo "Custom action"
```

## Support

For issues with CI/CD:
1. Check workflow logs in Actions tab
2. Review this documentation
3. Check GitHub Actions documentation
4. Open an issue in the repository
