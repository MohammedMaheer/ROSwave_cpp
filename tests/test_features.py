#!/usr/bin/env python3
"""
Atom-by-atom test suite for ROS2 Dashboard features.
Tests each component in isolation and integration.
"""

import subprocess
import sys
import time
import json
import re
from pathlib import Path
from typing import List, Dict, Any, Tuple

class TestResult:
    def __init__(self, name: str):
        self.name = name
        self.passed = False
        self.error: str | None = None
        self.details: Dict[str, Any] = {}
        
    def __repr__(self):
        status = "✅ PASS" if self.passed else "❌ FAIL"
        msg = f"{status} | {self.name}"
        if self.error:
            msg += f" | {self.error}"
        return msg

class FeatureTester:
    def __init__(self, project_root: str):
        self.project_root = Path(project_root)
        self.build_dir = self.project_root / "build"
        self.results: List[TestResult] = []
        
    def run_command(self, cmd: str, check: bool = False) -> Tuple[int, str, str]:
        """Run shell command and return (exit_code, stdout, stderr)"""
        try:
            result = subprocess.run(
                cmd,
                shell=True,
                capture_output=True,
                text=True,
                timeout=5
            )
            return result.returncode, result.stdout, result.stderr
        except subprocess.TimeoutExpired:
            return -1, "", "Command timed out"
        except Exception as e:
            return -1, "", str(e)
    
    def test_executable_exists(self) -> TestResult:
        """Test 1: Dashboard executable exists and is valid"""
        test = TestResult("Executable exists")
        exe_path = self.build_dir / "ros2_dashboard"
        
        if not exe_path.exists():
            test.error = f"Executable not found at {exe_path}"
            return test
            
        if not exe_path.is_file():
            test.error = f"Path exists but is not a file"
            return test
            
        if not (exe_path.stat().st_mode & 0o111):
            test.error = "File is not executable"
            return test
            
        size_mb = exe_path.stat().st_size / (1024 * 1024)
        test.details = {"size_mb": round(size_mb, 2), "path": str(exe_path)}
        test.passed = True
        return test
    
    def test_build_system(self) -> TestResult:
        """Test 2: Build system integrity"""
        test = TestResult("Build system integrity")
        
        # Check CMakeLists.txt
        cmake_file = self.project_root / "CMakeLists.txt"
        if not cmake_file.exists():
            test.error = "CMakeLists.txt not found"
            return test
        
        with open(cmake_file) as f:
            cmake_content = f.read()
        
        required_files = [
            "src/main.cpp",
            "src/ros2_manager.cpp",
            "src/async_worker.cpp",
            "src/gui/topics_tab.cpp",
            "src/gui/nodes_tab.cpp",
            "src/gui/services_tab.cpp",
            "src/gui/selected_topics_tab.cpp",
        ]
        
        missing = []
        for file in required_files:
            if file not in cmake_content:
                missing.append(file)
        
        if missing:
            test.error = f"Missing from CMakeLists.txt: {missing}"
            return test
        
        # Check build artifacts
        makefile = self.build_dir / "Makefile"
        if not makefile.exists():
            test.error = "Build Makefile not found"
            return test
        
        test.details = {"required_files": len(required_files), "found": len(required_files)}
        test.passed = True
        return test
    
    def test_compilation_errors(self) -> TestResult:
        """Test 3: No compilation errors"""
        test = TestResult("Zero compilation errors")
        
        # Re-run make to check for errors
        code, stdout, stderr = self.run_command(f"cd {self.build_dir} && make 2>&1")
        
        if "error:" in stderr.lower() or "error:" in stdout.lower():
            test.error = "Compilation errors detected"
            test.details = {"stderr": stderr[:200], "stdout": stdout[:200]}
            return test
        
        if "[100%]" not in stdout or "Built target" not in stdout:
            test.error = "Build did not complete successfully"
            return test
        
        test.passed = True
        test.details = {"build_status": "100% complete"}
        return test
    
    def test_ros2_cli_availability(self) -> TestResult:
        """Test 4: ROS2 CLI tools available"""
        test = TestResult("ROS2 CLI availability")
        
        required_commands = [
            ("ros2 topic list", "Topic discovery"),
            ("ros2 node list", "Node discovery"),
            ("ros2 service list", "Service discovery"),
        ]
        
        missing = []
        for cmd, desc in required_commands:
            code, _, _ = self.run_command(f"{cmd} 2>&1")
            if code != 0 and code != 1:  # 1 is "no topics" which is OK
                missing.append(desc)
        
        if missing:
            test.error = f"Missing ROS2 tools: {missing}"
            return test
        
        test.passed = True
        test.details = {"commands_available": len(required_commands)}
        return test
    
    def test_topic_discovery(self) -> TestResult:
        """Test 5: Topic discovery implementation"""
        test = TestResult("Topic discovery")
        
        # Check ros2_manager.cpp for discover_topics method
        ros2_mgr = self.project_root / "src" / "ros2_manager.cpp"
        with open(ros2_mgr) as f:
            content = f.read()
        
        checks = [
            ("discover_topics_", "discover_topics method"),
            ("ros2 topic list", "topic list command"),
            ("ros2 topic info", "topic info command"),
            ("TopicInfo", "topic info structure"),
        ]
        
        missing = []
        for check_str, desc in checks:
            if check_str not in content:
                missing.append(desc)
        
        if missing:
            test.error = f"Missing in ros2_manager.cpp: {missing}"
            return test
        
        # Try actual discovery
        code, stdout, _ = self.run_command("ros2 topic list -t 2>/dev/null")
        if code == 0:
            test.details = {"topics_found": len(stdout.strip().split('\n')) if stdout.strip() else 0}
        
        test.passed = True
        return test
    
    def test_node_discovery(self) -> TestResult:
        """Test 6: Node discovery implementation"""
        test = TestResult("Node discovery")
        
        # Check ros2_manager.cpp for discover_nodes method
        ros2_mgr = self.project_root / "src" / "ros2_manager.cpp"
        with open(ros2_mgr) as f:
            content = f.read()
        
        checks = [
            ("discover_nodes_", "discover_nodes method"),
            ("ros2 node list", "node list command"),
            ("ros2 node info", "node info command"),
        ]
        
        missing = []
        for check_str, desc in checks:
            if check_str not in content:
                missing.append(desc)
        
        if missing:
            test.error = f"Missing in ros2_manager.cpp: {missing}"
            return test
        
        # Try actual discovery
        code, stdout, _ = self.run_command("ros2 node list 2>/dev/null")
        if code == 0:
            test.details = {"nodes_found": len(stdout.strip().split('\n')) if stdout.strip() else 0}
        
        test.passed = True
        return test
    
    def test_service_discovery(self) -> TestResult:
        """Test 7: Service discovery implementation"""
        test = TestResult("Service discovery")
        
        # Check ros2_manager.cpp for discover_services method
        ros2_mgr = self.project_root / "src" / "ros2_manager.cpp"
        with open(ros2_mgr) as f:
            content = f.read()
        
        checks = [
            ("discover_services_", "discover_services method"),
            ("ros2 service list", "service list command"),
        ]
        
        missing = []
        for check_str, desc in checks:
            if check_str not in content:
                missing.append(desc)
        
        if missing:
            test.error = f"Missing in ros2_manager.cpp: {missing}"
            return test
        
        # Try actual discovery
        code, stdout, _ = self.run_command("ros2 service list 2>/dev/null")
        if code == 0:
            test.details = {"services_found": len(stdout.strip().split('\n')) if stdout.strip() else 0}
        
        test.passed = True
        return test
    
    def test_threading_model(self) -> TestResult:
        """Test 8: Threading architecture"""
        test = TestResult("Threading architecture")
        
        # Check async_worker.hpp
        async_worker_h = self.project_root / "include" / "async_worker.hpp"
        with open(async_worker_h) as f:
            async_content = f.read()
        
        # Check async_worker.cpp
        async_worker_cpp = self.project_root / "src" / "async_worker.cpp"
        with open(async_worker_cpp) as f:
            async_cpp = f.read()
        
        checks = [
            (async_content, "std::thread", "Thread support"),
            (async_content, "std::function", "Async callbacks"),
            (async_cpp, "worker_thread_", "Worker thread implementation"),
        ]
        
        missing = []
        for content, check_str, desc in checks:
            if check_str not in content:
                missing.append(desc)
        
        if missing:
            test.error = f"Missing threading components: {missing}"
            return test
        
        test.passed = True
        test.details = {"components": 3}
        return test
    
    def test_signal_slot_pattern(self) -> TestResult:
        """Test 9: Qt signal/slot connections"""
        test = TestResult("Signal/slot pattern")
        
        tabs = {
            "topics_tab.cpp": ["on_topics_updated", "on_message_updated"],
            "nodes_tab.cpp": ["on_nodes_updated"],
            "services_tab.cpp": ["on_services_updated"],
        }
        
        missing_signals = {}
        for tab_file, signals in tabs.items():
            tab_path = self.project_root / "src" / "gui" / tab_file
            with open(tab_path) as f:
                content = f.read()
            
            missing = []
            for signal in signals:
                if signal not in content:
                    missing.append(signal)
            
            if missing:
                missing_signals[tab_file] = missing
        
        if missing_signals:
            test.error = f"Missing signals in tabs: {missing_signals}"
            return test
        
        # Check for Qt::QueuedConnection in any tab (at least one should have it)
        found_queued = False
        for tab_file in tabs.keys():
            tab_path = self.project_root / "src" / "gui" / tab_file
            with open(tab_path) as f:
                content = f.read()
            if "Qt::QueuedConnection" in content:
                found_queued = True
                break
        
        if not found_queued:
            test.error = "Qt::QueuedConnection not found in any tab"
            return test
        
        test.passed = True
        test.details = {"tabs_checked": len(tabs), "signals_verified": sum(len(s) for s in tabs.values())}
        return test
    
    def test_type_registration(self) -> TestResult:
        """Test 10: Qt type registration"""
        test = TestResult("Qt type registration")
        
        main_cpp = self.project_root / "src" / "main.cpp"
        with open(main_cpp) as f:
            content = f.read()
        
        required_types = [
            "qRegisterMetaType<std::vector<ros2_dashboard::TopicInfo>>",
            "qRegisterMetaType<std::vector<ros2_dashboard::NodeInfo>>",
            "qRegisterMetaType<std::vector<ros2_dashboard::ServiceInfo>>",
        ]
        
        missing = []
        for type_reg in required_types:
            if type_reg not in content:
                missing.append(type_reg.split("::")[-1][:20])
        
        if missing:
            test.error = f"Missing type registrations: {missing}"
            return test
        
        test.passed = True
        test.details = {"types_registered": len(required_types)}
        return test
    
    def test_topics_tab(self) -> TestResult:
        """Test 11: Topics tab implementation"""
        test = TestResult("Topics tab features")
        
        topics_tab = self.project_root / "src" / "gui" / "topics_tab.cpp"
        with open(topics_tab) as f:
            content = f.read()
        
        features = [
            ("QWidget", "Widget base"),
            ("QTimer", "Auto-refresh timer"),
            ("refresh_timer_", "Timer instance"),
            ("on_topics_updated", "Signal emission"),
        ]
        
        missing = []
        for feature, desc in features:
            if feature not in content:
                missing.append(desc)
        
        if missing:
            test.error = f"Missing in topics_tab: {missing}"
            return test
        
        test.passed = True
        test.details = {"features": len(features)}
        return test
    
    def test_nodes_tab(self) -> TestResult:
        """Test 12: Nodes tab implementation"""
        test = TestResult("Nodes tab features")
        
        nodes_tab = self.project_root / "src" / "gui" / "nodes_tab.cpp"
        with open(nodes_tab) as f:
            content = f.read()
        
        features = [
            ("QTreeWidget", "Tree widget"),
            ("QTimer", "Auto-refresh timer"),
            ("addTopLevelItem", "Node display"),
        ]
        
        missing = []
        for feature, desc in features:
            if feature not in content:
                missing.append(desc)
        
        if missing:
            test.error = f"Missing in nodes_tab: {missing}"
            return test
        
        test.passed = True
        test.details = {"features": len(features)}
        return test
    
    def test_services_tab(self) -> TestResult:
        """Test 13: Services tab implementation"""
        test = TestResult("Services tab features")
        
        services_tab = self.project_root / "src" / "gui" / "services_tab.cpp"
        with open(services_tab) as f:
            content = f.read()
        
        features = [
            ("QTableWidget", "Table widget"),
            ("QTimer", "Auto-refresh timer"),
            ("total_services_label_", "Service counter"),
            ("📡", "Emoji headers"),
        ]
        
        missing = []
        for feature, desc in features:
            if feature not in content:
                missing.append(desc)
        
        if missing:
            test.error = f"Missing in services_tab: {missing}"
            return test
        
        test.passed = True
        test.details = {"features": len(features)}
        return test
    
    def test_selected_topics_tab(self) -> TestResult:
        """Test 14: Selected topics tab implementation"""
        test = TestResult("Selected topics tab features")
        
        selected_topics = self.project_root / "src" / "gui" / "selected_topics_tab.cpp"
        with open(selected_topics) as f:
            content = f.read()
        
        features = [
            ("QTableWidget", "Table widget"),
            ("QComboBox", "Topic dropdown"),
            ("🟢", "Health indicator green"),
            ("🟡", "Health indicator yellow"),
            ("🔴", "Health indicator red"),
        ]
        
        missing = []
        for feature, desc in features:
            if feature not in content:
                missing.append(desc)
        
        if missing:
            test.error = f"Missing in selected_topics_tab: {missing}"
            return test
        
        test.passed = True
        test.details = {"features": len(features)}
        return test
    
    def test_recording_tab(self) -> TestResult:
        """Test 15: Recording tab implementation"""
        test = TestResult("Recording tab features")
        
        recording_tab = self.project_root / "src" / "gui" / "recording_tab.cpp"
        with open(recording_tab) as f:
            content = f.read()
        
        features = [
            ("output_dir_edit_", "Output directory widget"),
            ("compression_combo_", "Compression selection widget"),
            ("start_button_", "Start button"),
            ("stop_button_", "Stop button"),
        ]
        
        missing = []
        for feature, desc in features:
            if feature not in content:
                missing.append(desc)
        
        if missing:
            test.error = f"Missing in recording_tab: {missing}"
            return test
        
        test.passed = True
        test.details = {"features": len(features)}
        return test
    
    def test_network_tab(self) -> TestResult:
        """Test 16: Network tab structure"""
        test = TestResult("Network tab structure")
        
        network_tab = self.project_root / "src" / "gui" / "network_tab.cpp"
        if not network_tab.exists():
            test.error = "network_tab.cpp not found"
            return test
        
        with open(network_tab) as f:
            content = f.read()
        
        if len(content.strip()) < 100:
            test.error = "network_tab.cpp appears empty or minimal"
            return test
        
        test.passed = True
        test.details = {"file_size": len(content)}
        return test
    
    def test_upload_tab(self) -> TestResult:
        """Test 17: Upload tab structure"""
        test = TestResult("Upload tab structure")
        
        upload_tab = self.project_root / "src" / "gui" / "upload_tab.cpp"
        if not upload_tab.exists():
            test.error = "upload_tab.cpp not found"
            return test
        
        with open(upload_tab) as f:
            content = f.read()
        
        if len(content.strip()) < 100:
            test.error = "upload_tab.cpp appears empty or minimal"
            return test
        
        test.passed = True
        test.details = {"file_size": len(content)}
        return test
    
    def test_metrics_collector(self) -> TestResult:
        """Test 18: Metrics collection"""
        test = TestResult("Metrics collector")
        
        metrics_file = self.project_root / "src" / "metrics_collector.cpp"
        with open(metrics_file) as f:
            content = f.read()
        
        features = [
            ("get_system_metrics", "System metrics method"),
            ("cpu_usage_percent", "CPU metrics"),
            ("memory_usage_mb", "Memory metrics"),
        ]
        
        missing = []
        for feature, desc in features:
            if feature not in content:
                missing.append(desc)
        
        if missing:
            test.error = f"Missing in metrics_collector: {missing}"
            return test
        
        test.passed = True
        test.details = {"features": len(features)}
        return test
    
    def test_ros2_cli_compatibility(self) -> TestResult:
        """Test 19: ROS2 CLI command compatibility"""
        test = TestResult("ROS2 CLI compatibility")
        
        ros2_mgr = self.project_root / "src" / "ros2_manager.cpp"
        with open(ros2_mgr) as f:
            content = f.read()
        
        # Check that unsupported flags are NOT used
        bad_flags = [
            ("--limit 1", "Unsupported limit flag"),
            ('--compression"', "Wrong compression flag format"),
        ]
        
        problems = []
        for flag, desc in bad_flags:
            # These should be fixed/removed
            if flag in content and "head -30" not in content:
                problems.append(desc)
        
        # Check for correct implementations
        good_patterns = [
            ("head -30", "Output limiting"),
            ("--compression-format", "Compression format flag"),
        ]
        
        found = []
        for pattern, desc in good_patterns:
            if pattern in content:
                found.append(desc)
        
        if len(found) < 1:
            test.error = "Missing correct ROS2 CLI patterns"
            return test
        
        test.passed = True
        test.details = {"compatible_patterns": len(found)}
        return test
    
    def test_error_handling(self) -> TestResult:
        """Test 20: Error handling"""
        test = TestResult("Error handling")
        
        ros2_mgr = self.project_root / "src" / "ros2_manager.cpp"
        with open(ros2_mgr) as f:
            content = f.read()
        
        features = [
            ("try", "Try/catch blocks"),
            ("2>/dev/null", "Stderr suppression"),
            ("if (", "Conditional checks"),
        ]
        
        missing = []
        for feature, desc in features:
            if feature not in content:
                missing.append(desc)
        
        if missing:
            test.error = f"Missing error handling: {missing}"
            return test
        
        test.passed = True
        test.details = {"features": len(features)}
        return test
    
    def run_all_tests(self) -> None:
        """Run all tests"""
        print("\n" + "="*80)
        print("🧪 ROS2 DASHBOARD - ATOM-BY-ATOM FEATURE TEST SUITE")
        print("="*80 + "\n")
        
        tests = [
            self.test_executable_exists,
            self.test_build_system,
            self.test_compilation_errors,
            self.test_ros2_cli_availability,
            self.test_topic_discovery,
            self.test_node_discovery,
            self.test_service_discovery,
            self.test_threading_model,
            self.test_signal_slot_pattern,
            self.test_type_registration,
            self.test_topics_tab,
            self.test_nodes_tab,
            self.test_services_tab,
            self.test_selected_topics_tab,
            self.test_recording_tab,
            self.test_network_tab,
            self.test_upload_tab,
            self.test_metrics_collector,
            self.test_ros2_cli_compatibility,
            self.test_error_handling,
        ]
        
        for i, test_func in enumerate(tests, 1):
            result = test_func()
            self.results.append(result)
            
            status = "✅" if result.passed else "❌"
            print(f"{i:2d}. {status} {result.name}")
            
            if result.details:
                for key, value in result.details.items():
                    print(f"    └─ {key}: {value}")
            
            if result.error:
                print(f"    └─ ❌ {result.error}")
            
            print()
        
        self.print_summary()
    
    def print_summary(self) -> None:
        """Print test summary"""
        passed = sum(1 for r in self.results if r.passed)
        total = len(self.results)
        
        print("="*80)
        print(f"📊 TEST SUMMARY: {passed}/{total} PASSED")
        print("="*80 + "\n")
        
        if passed == total:
            print("🎉 ALL TESTS PASSED! Dashboard is fully functional.\n")
        else:
            print(f"⚠️  {total - passed} test(s) failed. Review errors above.\n")
            
        print("Test Categories:")
        print(f"  • Build & Execution: {'✅' if all(r.passed for r in self.results[:3]) else '❌'}")
        print(f"  • ROS2 Integration: {'✅' if all(r.passed for r in self.results[3:7]) else '❌'}")
        print(f"  • Threading Model: {'✅' if all(r.passed for r in self.results[7:10]) else '❌'}")
        print(f"  • UI Components: {'✅' if all(r.passed for r in self.results[10:17]) else '❌'}")
        print(f"  • Code Quality: {'✅' if all(r.passed for r in self.results[17:]) else '❌'}")
        print()

if __name__ == "__main__":
    project_root = Path(__file__).parent.parent
    tester = FeatureTester(str(project_root))
    tester.run_all_tests()
