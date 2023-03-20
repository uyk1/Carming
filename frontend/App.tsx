/**
 * Sample React Native App
 * https://github.com/facebook/react-native
 *
 * @format
 */

import {NavigationContainer} from '@react-navigation/native';
import {useState} from 'react';
import L1_RootStackNavigator from './src/navigations/L1_RootStackNavigator';
import LaunchScreen from './src/screens/LaunchScreen';

function App(): JSX.Element {
  const [isLoaded, setIsLoaded] = useState(false);

  setTimeout(() => {
    setIsLoaded(true);
  }, 2000);

  return (
    <>
      {isLoaded ? (
        <NavigationContainer>
          <L1_RootStackNavigator />
        </NavigationContainer>
      ) : (
        <LaunchScreen />
      )}
    </>
  );
}

export default App;
