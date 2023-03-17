/**
 * Sample React Native App
 * https://github.com/facebook/react-native
 *
 * @format
 */

import {NavigationContainer} from '@react-navigation/native';
import L1_RootStackNavigator from './src/navigations/L1_RootStackNavigator';

function App(): JSX.Element {
  return (
    <>
      <NavigationContainer>
        <L1_RootStackNavigator />
      </NavigationContainer>
    </>
  );
}

export default App;
