using UnityEngine;
using UnityEngine.TestTools;
using NUnit.Framework;
using System.Collections;
#if ENABLE_INPUT_SYSTEM && NEW_INPUT_SYSTEM_INSTALLED
using UnityEngine.InputSystem;
#endif

// This package does not contain any runtim components, therefore
// this test is only a placeholder.


[TestFixture]
class RuntimeExampleTest {

	[SetUp]
	public void SetUp()
	{
#if ENABLE_INPUT_SYSTEM && NEW_INPUT_SYSTEM_INSTALLED
		if (Keyboard.current == null)
		{
			InputSystem.AddDevice<Keyboard>();
		}
#endif
	}
	
	[Test]
	public void PlayModeSampleTestSimplePasses() 
	{
		// Use the Assert class to test conditions.
	}

}
